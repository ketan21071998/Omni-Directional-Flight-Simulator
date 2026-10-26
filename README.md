# Omni-Directional Flight Simulator

This project is a small flight simulation for a quadcopter with independently tilting motors. Each motor has two rotary joints, allowing its thrust direction to change without relying entirely on tilting the vehicle body.

The simulator is written in C++. It handles the vehicle dynamics, mission targets, TCP communication, and 3D rendering. A separate Python controller reads the simulated vehicle state and sends motor thrust and joint-angle commands back to the simulator.

The project currently runs on Windows. The build and run steps below assume PowerShell and a Visual Studio C++ toolchain.

## What is implemented

- Four motors, each with a thrust command and two joint-angle commands (12 actuator values in total).
- Rigid-body simulation updated at 120 Hz.
- TCP telemetry and control exchange at 50 Hz, using port 8080 by default.
- Python guidance using position and velocity feedback, target-state estimation with a Kalman filter, and thrust-direction calculations.
- Roll, pitch, and yaw corrections through motor joint commands.
- Four fixed-target mission stages followed by moving-target stages.
- SFML/OpenGL display using STL models for the vehicle parts.

This is a simulation project, not flight-tested autopilot software.

## Files

```text
Omni-Directional-Flight-Simulator/
├── CMakeLists.txt
├── main.cpp
├── physics.h
├── renderer.h
├── stl_mesh.h
├── tcp_server.h
├── controller.py
├── models/
│   ├── BODY.stl
│   ├── arm_1.stl
│   ├── arm_2.stl
│   ├── motor.stl
│   └── propeller.stl
├── assets/
│   └── arial.ttf              # optional; see font note below
├── requirements.txt
├── .gitignore
└── README.md
```

The model names and capitalization matter. The renderer loads `models/BODY.stl`, `models/arm_1.stl`, `models/arm_2.stl`, and `models/propeller.stl`. `motor.stl` can be kept with the other CAD files, but the current renderer does not load it.

The renderer also looks for `assets/arial.ttf` and then falls back to `C:/Windows/Fonts/arial.ttf`. Do not redistribute a font file unless its license allows it. On a normal Windows installation, the system font may be sufficient.

## Requirements

- Windows 10 or 11, 64-bit
- Visual Studio 2022 with **Desktop development with C++** installed
- CMake 3.20 or newer
- Git and [vcpkg](https://github.com/microsoft/vcpkg) to install SFML 3
- Python 3 with pip
- NumPy (`requirements.txt`)
- OpenGL support through your graphics driver

The C++ project uses C++17 and links against SFML Graphics, Window and System, OpenGL, GLU, and Windows sockets.

## Build on Windows

Open PowerShell in the repository root (the directory containing `CMakeLists.txt`).

**1. Install SFML 3 using vcpkg.** If vcpkg is not already installed:

```powershell
git clone https://github.com/microsoft/vcpkg.git C:\vcpkg
C:\vcpkg\bootstrap-vcpkg.bat
C:\vcpkg\vcpkg.exe install sfml:x64-windows
```

If you installed vcpkg elsewhere, use that path in the next command.

**2. Configure and build.**

```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake"
cmake --build build --config Release
```

The expected executable is:

```text
build/Release/DroneSimulator.exe
```

This location matters: `controller.py` launches the executable from exactly this path. Do not move the executable unless you also change the controller.

**3. Install the Python dependency.**

```powershell
python -m pip install -r requirements.txt
```

**4. Run the controller from the repository root.**

```powershell
python controller.py
```

The Python script starts `build/Release/DroneSimulator.exe`, waits briefly, then connects to the simulator over localhost TCP port 8080. You do not need to start the executable separately.

You can specify another port if needed:

```powershell
python controller.py 8081
```

## Working directory and 3D models

Run `python controller.py` from the project root. The controller starts the simulator with that folder as its working directory, and the renderer looks for STL files using relative paths such as `models/BODY.stl`.

If you start `DroneSimulator.exe` manually from inside `build/Release`, the renderer may not find the models. Run it from the repository root instead:

```powershell
.\build\Release\DroneSimulator.exe
```

Running the executable alone opens the simulation, but the Python controller is needed to send autonomous control commands.

## Using the simulation

The display shows the drone, its current target, and flight information. The simulator progresses through the mission targets when the vehicle reaches them. The Python controller continuously reads state information and sends four PWM values plus four angles for each of the two motor joints.

- **Left mouse button + drag:** rotate the camera
- **Mouse wheel:** zoom
- **R:** reset the simulation
- **Esc:** close the window

The TCP server listens on `127.0.0.1` by default. The controller and simulator are intended to run on the same machine.

## Troubleshooting

**CMake cannot find SFML:** Check that `sfml:x64-windows` is installed and the `CMAKE_TOOLCHAIN_FILE` path points to your vcpkg installation. Use the x64 Visual Studio generator.

**Python reports that `DroneSimulator.exe` cannot be found:** Build the Release configuration and check that `build/Release/DroneSimulator.exe` exists. A Debug build produces a different directory.

**The window opens but the vehicle is missing:** Check the STL filenames and the `models/` folder. The mesh loader expects binary STL files. Also make sure you launched the project from its root directory.

**The HUD text is missing:** The renderer could not load `assets/arial.ttf` or the Windows Arial font. The simulation may still run without the text overlay.

**Connection refused on port 8080:** Check the simulator console for startup errors and make sure another process is not already using that port.

**The program starts but crashes due to missing DLLs:** Check that the required SFML runtime libraries are available to the executable. This can depend on how SFML was installed and linked.

## Notes

The source code is kept in its existing form. The instructions above describe the expected Windows build layout; they have not been verified through a complete Windows build in this environment.

The `build/` directory and generated binaries should not be committed to Git. Keep the original STL models in `models/` when publishing the repository, provided you have permission to redistribute them.
