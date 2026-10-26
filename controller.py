import subprocess
import socket
import json
import time
import sys
import os
import numpy as np

# ================= PARAMETERS =================
mass = 1.5
g = 9.81
max_thrust_per_motor = 20.0

# Guidance (unchanged from your original)
Kp = 2.2
Kd = 3.2
NAV_GAIN = 6.5
MAX_ACCEL = 15.5
MAX_TILT_DEG = 35.0
MAX_TILT_RAD = np.radians(MAX_TILT_DEG)

alpha = 0.4
prev_acc_cmd = np.zeros(3)

# Attitude leveling (roll/pitch back to 0) — small differential added
# on top of the common vectored-thrust direction. Since thrust
# vectoring handles translation, this loop's only job is to damp out
# any residual body tilt, not to steer the vehicle.
K_LEVEL_P = np.array([0.6, 0.6])   # [pitch, roll] proportional gain (deg output per deg error)
K_LEVEL_D = np.array([0.15, 0.15]) # [pitch, roll] derivative gain (deg output per rad/s)

# Yaw control — since all 4 motors pointing the same direction produces
# zero net torque (by arm symmetry), yaw needs an explicit differential
# twist on joint1 across motors.
YAW_DES = 0.0
K_YAW_P = 0.8   # deg of joint1 differential per deg of yaw error
K_YAW_D = 0.3   # deg of joint1 differential per rad/s of yaw rate

JOINT_LIMIT_DEG = 90.0

# ================= KALMAN FILTER =================
class TargetKalman:
    def __init__(self):
        self.x = np.zeros(6)  # [x,y,z,vx,vy,vz]
        self.P = np.eye(6) * 10.0

        q_pos = 0.1
        q_vel = 1.0
        self.Q = np.diag([q_pos, q_pos, q_pos, q_vel, q_vel, q_vel])

        r_pos = 0.5
        self.R = np.eye(3) * r_pos

        self.H = np.zeros((3, 6))
        self.H[:, :3] = np.eye(3)

        self.initialized = False

    def predict(self, dt):
        F = np.eye(6)
        F[0,3] = dt
        F[1,4] = dt
        F[2,5] = dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z):
        if not self.initialized:
            self.x[:3] = z
            self.initialized = True
            return

        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    def step(self, z, dt):
        self.predict(dt)
        self.update(z)
        return self.x[:3], self.x[3:]

# ================= UTIL =================
def clamp01(x):
    return np.clip(x, 0.0, 1.0)

def clamp_joint(deg):
    return float(np.clip(deg, -JOINT_LIMIT_DEG, JOINT_LIMIT_DEG))

def wrap180(d):
    return (d + 180.0) % 360.0 - 180.0

def euler_to_R(pitch, yaw, roll):
    # Matches physics.h euler_to_R exactly: pitch->Rx, yaw->Ry,
    # roll->Rz, R = Rz @ Ry @ Rx
    Rx = np.array([
        [1,0,0],
        [0,np.cos(pitch),-np.sin(pitch)],
        [0,np.sin(pitch), np.cos(pitch)]
    ])

    Ry = np.array([
        [np.cos(yaw),0,-np.sin(yaw)],
        [0,1,0],
        [np.sin(yaw),0,np.cos(yaw)]
    ])

    Rz = np.array([
        [np.cos(roll),-np.sin(roll),0],
        [np.sin(roll), np.cos(roll),0],
        [0,0,1]
    ])

    return Rz @ Ry @ Rx

def read_state(sock, buffer):
    chunk = sock.recv(4096).decode("utf-8")
    buffer += chunk

    while "\n" in buffer:
        line, buffer = buffer.split("\n",1)
        if line.strip():
            return json.loads(line), buffer

    return None, buffer

# ================= GUIDANCE (unchanged) =================
def guidance(pos, vel, target, target_vel):
    r = target - pos
    v_rel = target_vel - vel

    acc_nav = Kp * r + Kd * v_rel

    dist = np.linalg.norm(r)
    if dist > 10.0:
        r_hat = r / (dist + 1e-6)
        omega_los = np.cross(r, v_rel) / (dist**2 + 1e-6)
        Vc = -np.dot(r, v_rel) / (dist + 1e-6)

        acc_nav += NAV_GAIN * Vc * np.cross(omega_los, r_hat)

    nav_norm = np.linalg.norm(acc_nav)
    if nav_norm > MAX_ACCEL:
        acc_nav = acc_nav / nav_norm * MAX_ACCEL

    acc_cmd = acc_nav + np.array([0, g, 0])

    return acc_cmd, r

# ================= THRUST-VECTOR INVERSE KINEMATICS =================
# Given a desired thrust direction in the MOTOR'S OWN body-local frame
# (unit vector, EUS: x=right, y=up, z=forward-ish per physics.h), solve
# the (joint1, joint2) pair that makes motor_direction(joint1, joint2)
# in physics.h point along that direction.
#
# physics.h's motor_direction() starts from local (0,1,0) [straight up],
# rotates by joint1 around Z, then by joint2 around X. Working the
# rotation through algebraically:
#   dir = ( -sin(j1),  cos(j1)*cos(j2),  cos(j1)*sin(j2) )
# Inverting:
#   j1 = -asin(dx)
#   j2 = atan2(dz, dy)     (the cos(j1) factor cancels inside atan2)
def solve_joint_angles(dir_body):
    dx, dy, dz = dir_body
    dx = float(np.clip(dx, -1.0, 1.0))

    j1_rad = -np.arcsin(dx)
    j2_rad = np.arctan2(dz, dy)

    j1_deg = clamp_joint(np.degrees(j1_rad))
    j2_deg = clamp_joint(np.degrees(j2_rad))

    return j1_deg, j2_deg

# ================= MAIN =================
def main():
    global prev_acc_cmd

    last_time = None
    port = 8080 if len(sys.argv) < 2 else int(sys.argv[1])

    script_dir = os.path.dirname(os.path.abspath(__file__))
    sim_exe = os.path.join(script_dir, "build/Release/DroneSimulator.exe")

    print("Starting simulation...")
    sim_process = subprocess.Popen([sim_exe, str(port)], cwd=script_dir)
    time.sleep(3)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(("localhost", port))
    sock.settimeout(0.2)

    print("Connected.")

    buffer = ""
    kf = TargetKalman()

    try:
        while True:
            state, buffer = read_state(sock, buffer)
            if state is None:
                continue

            pos = np.array(state["position"])
            vel = np.array(state["velocity"])
            target_meas = np.array(state["target_position"])

            pitch, yaw, roll = np.radians(state["rotation"])
            ang_vel = np.array(state["angular_vel_body"])  # [wx(pitch-rate), wy(yaw-rate), wz(roll-rate)]

            now = time.time()
            dt = 0.05 if last_time is None else now - last_time
            last_time = now
            dt = np.clip(dt, 0.005, 0.2)

            # ===== Kalman Filter =====
            target, target_vel = kf.step(target_meas, dt)

            # ===== Guidance: desired WORLD-frame acceleration (already
            # includes gravity feed-forward, so mass*acc_cmd is the
            # desired WORLD-frame thrust vector) =====
            acc_cmd, r = guidance(pos, vel, target, target_vel)

            acc_cmd = (1 - alpha)*prev_acc_cmd + alpha*acc_cmd
            prev_acc_cmd = acc_cmd.copy()

            # Optional horizontal/vertical shaping, same spirit as your
            # original tilt limit — here it limits how "sideways" the
            # thrust vector can point relative to vertical, which caps
            # how hard the vehicle accelerates horizontally rather than
            # capping a body tilt angle (there's no body tilt to limit
            # anymore since motors vector independently of the airframe).
            vertical = acc_cmd[1]
            horizontal = np.array([acc_cmd[0], 0.0, acc_cmd[2]])
            hmag = np.linalg.norm(horizontal)
            max_horizontal = abs(vertical) * np.tan(MAX_TILT_RAD)
            if hmag > max_horizontal and hmag > 1e-6:
                horizontal *= max_horizontal / hmag
            acc_cmd = np.array([horizontal[0], vertical, horizontal[2]])

            thrust_world = mass * acc_cmd
            thrust_total = np.linalg.norm(thrust_world)
            thrust_total = np.clip(thrust_total, 0.0, 4.0 * max_thrust_per_motor)
            pwm_common = clamp01(thrust_total / (4.0 * max_thrust_per_motor))

            # ===== Convert desired thrust direction into BODY frame,
            # using CURRENT measured attitude — this is what lets thrust
            # vectoring compensate for any residual body tilt in
            # real time without needing the body itself to re-level
            # first. =====
            R = euler_to_R(pitch, yaw, roll)
            thrust_dir_world = thrust_world / (np.linalg.norm(thrust_world) + 1e-6)
            thrust_dir_body = R.T @ thrust_dir_world

            j1_common, j2_common = solve_joint_angles(thrust_dir_body)

            # ===== Roll/pitch leveling — small differential correction
            # added on top of the common vectored direction, so the
            # airframe itself relaxes back toward level over time
            # (purely cosmetic/stability; translation doesn't depend on
            # this since vectoring already compensates for tilt above). =====
            pitch_deg = np.degrees(pitch)
            roll_deg  = np.degrees(roll)
            pitch_rate = ang_vel[0]
            roll_rate  = ang_vel[2]

            level_pitch_cmd = -K_LEVEL_P[0]*pitch_deg - K_LEVEL_D[0]*np.degrees(pitch_rate)
            level_roll_cmd  = -K_LEVEL_P[1]*roll_deg  - K_LEVEL_D[1]*np.degrees(roll_rate)

            # ===== Yaw control — differential joint1 twist across
            # motors (alternating sign at diagonal corners). If yaw
            # goes the wrong direction on your build, flip the sign of
            # yaw_pattern below (motor mesh/torque sign conventions can
            # vary — see the sign-convention notes throughout this
            # project's renderer/physics work). =====
            yaw_deg = np.degrees(yaw)
            yaw_error = wrap180(YAW_DES - yaw_deg)
            yaw_rate = ang_vel[1]
            yaw_cmd_deg = K_YAW_P*yaw_error - K_YAW_D*np.degrees(yaw_rate)

            # Motor order matches physics.h / renderer.h: FR, FL, BR, BL
            yaw_pattern  = np.array([ 1, -1, -1,  1])
            pitch_pattern = np.array([ 1,  1, -1, -1])  # front vs back, via joint2
            roll_pattern  = np.array([ 1, -1,  1, -1])  # right vs left, via joint1

            joint1 = np.full(4, j1_common) + yaw_pattern*yaw_cmd_deg + roll_pattern*level_roll_cmd*0.3
            joint2 = np.full(4, j2_common) + pitch_pattern*level_pitch_cmd*0.3

            joint1 = np.clip(joint1, -JOINT_LIMIT_DEG, JOINT_LIMIT_DEG)
            joint2 = np.clip(joint2, -JOINT_LIMIT_DEG, JOINT_LIMIT_DEG)

            pwm = np.full(4, pwm_common)

            sock.send((json.dumps({
                "pwm": pwm.tolist(),
                "joint1": joint1.tolist(),
                "joint2": joint2.tolist(),
            }) + "\n").encode())

    except KeyboardInterrupt:
        print("\nShutting down...")

    finally:
        sock.close()
        sim_process.terminate()
        sim_process.wait()

if __name__ == "__main__":
    main()
