#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
constexpr double PI = 3.14159265358979323846;

#include <array>
#include <mutex>
#include <algorithm>
#include <chrono>
#include <vector>

// =========================================================
// Vec3 – minimal 3-D vector
// =========================================================
struct Vec3 {
    double x{0}, y{0}, z{0};

    Vec3() = default;
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s)      const { return {x*s,   y*s,   z*s};   }
    Vec3 operator/(double s)      const { return {x/s,   y/s,   z/s};   }
    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator*=(double s)     { x*=s;   y*=s;   z*=s;   return *this; }

    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return { y*o.z - z*o.y,
                 z*o.x - x*o.z,
                 x*o.y - y*o.x };
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
};
inline Vec3 operator*(double s, const Vec3& v){ return v*s; }

// =========================================================
// Mat3 – 3×3 rotation matrix (row-major)
// =========================================================
struct Mat3 {
    double m[3][3]{};

    static Mat3 identity() {
        Mat3 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=1; return r;
    }

    Vec3 operator*(const Vec3& v) const {
        return { m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
                 m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
                 m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z };
    }

    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for(int i=0;i<3;i++)
          for(int j=0;j<3;j++)
            for(int k=0;k<3;k++)
              r.m[i][j] += m[i][k]*o.m[k][j];
        return r;
    }

    Mat3 T() const {   // transpose
        Mat3 r;
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) r.m[i][j]=m[j][i];
        return r;
    }
};

// =========================================================
// Physical constants  (mirrors Python exactly)
// =========================================================
constexpr double MASS               = 1.5;
constexpr double GRAVITY            = 9.81;
constexpr double MAX_THRUST         = 20.0;
constexpr double MAX_RPM            = 20000.0;
constexpr double RPM_RESPONSE       = 0.08;
constexpr double ARM_LEN            = 0.25;
constexpr double DRAG_COEFF         = 0.01;
constexpr double ROT_DAMPING_120HZ  = 0.9965;
constexpr double SIM_HZ             = 120.0;
constexpr double DT                 = 1.0 / SIM_HZ;

// Single source of truth for "target reached" — used by BOTH the mission
// manager (physics.h) and the HUD (renderer.h) so they never disagree again.
constexpr double TARGET_RADIUS      = 1.0;

// Reaction torque from motor spin (yaw from RPM imbalance).
// 0.0 = disabled (matches original behavior: yaw comes only from gimbal
// tilt). Set > 0 if you want hover-yaw authority without tilting joints.
constexpr double YAW_TORQUE_COEFF   = 0.0;

inline Vec3 INERTIA() { return {0.02, 0.04, 0.02}; }

// =========================================================
// Clamp PWM [0,1]
// =========================================================
inline double clamp01(double v){ return std::clamp(v, 0.0, 1.0); }


// =========================================================
// Euler → Rotation matrix  (EUS, same order as Python)
// pitch→Rx  yaw→Ry  roll→Rz,  result = Rz*Ry*Rx
// =========================================================
inline Mat3 euler_to_R(double pitch_deg, double yaw_deg, double roll_deg)
{
    auto p = pitch_deg * PI/180.0;
    auto y = yaw_deg   * PI/180.0;
    auto r = roll_deg  * PI/180.0;

    Mat3 Rx; Rx.m[0][0]=1;
    Rx.m[1][1]= std::cos(p); Rx.m[1][2]=-std::sin(p);
    Rx.m[2][1]= std::sin(p); Rx.m[2][2]= std::cos(p);

    Mat3 Ry; Ry.m[1][1]=1;
    Ry.m[0][0]= std::cos(y); Ry.m[0][2]=-std::sin(y);
    Ry.m[2][0]= std::sin(y); Ry.m[2][2]= std::cos(y);

    Mat3 Rz; Rz.m[2][2]=1;
    Rz.m[0][0]= std::cos(r); Rz.m[0][1]=-std::sin(r);
    Rz.m[1][0]= std::sin(r); Rz.m[1][1]= std::cos(r);

    return Rz * Ry * Rx;
}

// =========================================================
// Wrap angle to [-180, 180]
// =========================================================
inline double wrap180(double d){
    d = std::fmod(d + 180.0, 360.0);
    if(d < 0) d += 360.0;
    return d - 180.0;
}

inline Vec3 motor_direction(
    double joint1_deg,
    double joint2_deg)
{
    double j1 = joint1_deg * PI / 180.0;
    double j2 = joint2_deg * PI / 180.0;

    Vec3 dir {0,1,0};

    // rotate around Z
    double x1 =
        dir.x * std::cos(j1) -
        dir.y * std::sin(j1);

    double y1 =
        dir.x * std::sin(j1) +
        dir.y * std::cos(j1);

    dir.x = x1;
    dir.y = y1;

    // rotate around X
    double y2 =
        dir.y * std::cos(j2) -
        dir.z * std::sin(j2);

    double z2 =
        dir.y * std::sin(j2) +
        dir.z * std::cos(j2);

    dir.y = y2;
    dir.z = z2;

    double n = dir.length();

    if(n > 1e-6)
        dir = dir / n;

    return dir;
}

// =========================================================
// Shared simulation state
// =========================================================
struct SimState {
    mutable std::mutex mtx;

    Vec3   position      {0, 2, 0};
    Vec3   velocity      {0, 0, 0};
    Vec3   accel_body    {0, 0, 0};
    Vec3   rotation      {0, 0, 0};   // pitch, yaw, roll  (degrees)
    Vec3   angular_vel   {0, 0, 0};   // body frame rad/s

    int mission_index = 0;
    int current_level = 0;

    // Each pointer is optional (nullptr = "not sent, leave unchanged").
    // This lets the TCP server apply partial commands instead of
    // discarding a whole line just because one field was missing.
    void set_actuators(
        const std::array<double,4>* pwm_cmd,
        const std::array<double,4>* j1_cmd,
        const std::array<double,4>* j2_cmd)
    {
        std::lock_guard<std::mutex> lk(mtx);

        if(pwm_cmd)
            for(int i=0;i<4;i++)
                pwm[i] = clamp01((*pwm_cmd)[i]);

        if(j1_cmd)
            for(int i=0;i<4;i++)
                joint1_deg[i] = std::clamp((*j1_cmd)[i], -90.0, 90.0);

        if(j2_cmd)
            for(int i=0;i<4;i++)
                joint2_deg[i] = std::clamp((*j2_cmd)[i], -90.0, 90.0);
    }

    std::vector<Vec3> mission_targets =
    {
        {10,10,10},     // Level 1
        {-10,5,10},    // Level 2
        {-10,8,-10},   // Level 3
        {10,10,-10},    // Level 4

        // Moving target levels
        {0,10,0},
        {0,10,0},
        {0,10,0}
    };

    Vec3 target {10,10,10};
    bool target_reached = false;

    std::array<double,4> pwm {0,0,0,0};
    std::array<double,4> rpm {0,0,0,0};

    // first rotary joint
    std::array<double,4> joint1_deg {0,0,0,0};

    // second rotary joint
    std::array<double,4> joint2_deg {0,0,0,0};

    //std::array<double,4> pwm {0,0,0,0};  // FR FL BR BL

    double sim_time{0.0};   // seconds since reset

    void reset()
    {
        std::lock_guard<std::mutex> lk(mtx);

        position    = {0,2,0};
        velocity    = {0,0,0};
        accel_body  = {0,0,0};
        rotation    = {0,0,0};
        angular_vel = {0,0,0};

        pwm         = {0,0,0,0};
        rpm         = {0,0,0,0};

        joint1_deg = {0,0,0,0};
        joint2_deg = {0,0,0,0};

        mission_index = 0;
        target = mission_targets[0];
        target_reached = false;


        sim_time = 0.0;
    }

};

// =========================================================
// Physics step  – call at 120 Hz
// =========================================================






inline void physics_step(SimState& s)
{
    std::lock_guard<std::mutex> lk(s.mtx);

    for(int i=0;i<4;i++)
    {
        double target_rpm =
            s.pwm[i] * MAX_RPM;

        s.rpm[i] +=
            (target_rpm - s.rpm[i]) *
            RPM_RESPONSE;
    }


    double thrusts[4] =
    {
        s.pwm[0] * MAX_THRUST,   // FR
        s.pwm[1] * MAX_THRUST,   // FL
        s.pwm[2] * MAX_THRUST,   // BR
        s.pwm[3] * MAX_THRUST    // BL
    };

    Mat3 R = euler_to_R(s.rotation.x, s.rotation.y, s.rotation.z);

    Vec3 total_force_body {0,0,0};

    for(int i=0;i<4;i++)
    {
        Vec3 dir =
            motor_direction(
                s.joint1_deg[i],
                s.joint2_deg[i]);

        Vec3 thrust =
            dir * thrusts[i];

        total_force_body += thrust;
    }

    // Force in world frame
    Vec3 force_world =
        R * total_force_body;
    Vec3 gravity     {0, -MASS*GRAVITY, 0};
    Vec3 drag        = s.velocity * (-DRAG_COEFF);

    Vec3 accel_world = (force_world + gravity + drag) * (1.0/MASS);

    s.velocity += accel_world * DT;
    s.position += s.velocity * DT;
    s.accel_body = R.T() * accel_world;

    // Motor positions in body frame
    Vec3 motor_pos[4] = {
        { ARM_LEN, 0,  -ARM_LEN},  // FR
        {-ARM_LEN, 0,  -ARM_LEN},  // FL
        { ARM_LEN, 0,   ARM_LEN},  // BR
        {-ARM_LEN, 0,   ARM_LEN}   // BL
    };

    Vec3 torque {0,0,0};
    for(int i=0;i<4;i++)
    {
        Vec3 dir =
            motor_direction(
                s.joint1_deg[i],
                s.joint2_deg[i]);

        Vec3 f =
            dir * thrusts[i];

        torque += motor_pos[i].cross(f);
    }

    // Optional reaction torque from motor spin direction (RPM imbalance
    // yaw). Disabled by default (YAW_TORQUE_COEFF == 0) so this preserves
    // the original behavior where yaw comes purely from gimbal tilt.
    // FR=+, FL=-, BR=-, BL=+ matches the spin directions drawn in renderer.h.
    if(YAW_TORQUE_COEFF != 0.0)
    {
        const int spin_dir[4] = {1, -1, -1, 1};
        for(int i=0;i<4;i++)
            torque.y += spin_dir[i] * YAW_TORQUE_COEFF * s.rpm[i];
    }

    Vec3 I = INERTIA();
    Vec3 ang_accel { torque.x/I.x, torque.y/I.y, torque.z/I.z };
    s.angular_vel += ang_accel * DT;

    // NOTE: since DT == 1/120 exactly, DT/(1/120) == 1.0 always, so the
    // original std::pow(...) call was a no-op dressed up to look
    // rate-independent. This is what it actually evaluated to:
    s.angular_vel *= ROT_DAMPING_120HZ;

    double deg = 180.0/PI;
    s.rotation.x = wrap180(s.rotation.x + s.angular_vel.x*DT*deg);
    s.rotation.y = wrap180(s.rotation.y + s.angular_vel.y*DT*deg);
    s.rotation.z = wrap180(s.rotation.z + s.angular_vel.z*DT*deg);

    // Ground collision
    if(s.position.y < 0.0){
        s.position.y = 0.0;
        if(s.velocity.y < 0) s.velocity.y = 0.0;
        s.angular_vel *= 0.5;
    }

    // =====================================================
    // Mission manager
    // =====================================================

    Vec3 diff =
    {
        s.position.x - s.target.x,
        s.position.y - s.target.y,
        s.position.z - s.target.z
    };

    // Distance-based check — matches the HUD's "TARGET REACHED!" text
    // exactly (both use TARGET_RADIUS), so the two no longer disagree.
    s.target_reached = diff.length() < TARGET_RADIUS;

    if(s.target_reached)
    {
        s.mission_index++;

        if(s.mission_index >= (int)s.mission_targets.size())
            s.mission_index = (int)s.mission_targets.size() - 1;

        s.target = s.mission_targets[s.mission_index];
    }

    // =====================================================
    // Moving target missions
    // =====================================================

    if(s.mission_index == 4)
    {
        // Move along Z axis
        s.target.z = 1.5 * s.sim_time;
    }

    if(s.mission_index == 5)
    {
        // Move along X axis
        s.target.x = 20.0 * std::sin(0.3 * s.sim_time);
    }

    if(s.mission_index == 6)
    {
        // Circle
        s.target.x = 20.0 * std::cos(0.3 * s.sim_time);
        s.target.z = 20.0 * std::sin(0.3 * s.sim_time);
    }

    s.sim_time += DT;
}
