# BotBoarduino CH3R PS2 (Lynxmotion Phoenix Port)

## Project Overview
This repository contains a modernized, Object-Oriented C++ implementation of the classic Lynxmotion Phoenix hexapod codebase. It is specifically configured for a circular hexapod (CH3R) utilizing a **BotBoarduino** (or Arduino Pro Mini / ATmega328P), an **SSC-32 Servo Controller**, and a **PS2 Wireless Controller**.

The codebase leverages highly optimized, 16-bit scaled integer math and `PROGMEM` lookup tables to execute complex 3D Inverse Kinematics (IK), 3D Forward Kinematics (FK), and gait generation strictly within a 50Hz (20ms) control loop on an 8-bit microcontroller, completely avoiding slow floating-point overhead.

## Hardware Requirements
* **Microcontroller:** Lynxmotion BotBoarduino (ATmega328P, 16MHz)
* **Servo Controller:** Lynxmotion SSC-32 (configured for Binary Mode `cSSC_BINARYMODE`)
* **Input:** PS2 Wireless Controller
* **Robot Chassis:** CH3R (Circular Hexapod 3-DOF, with optional 4-DOF support)

## PS2 Controller Mapping

### Common Controls (Global)
These controls work regardless of the active mode:
* **Start:** Turn the robot **On** (Stand up) or **Off** (Sit down and relax servos).
* **Triangle:** Quick toggle between **Stand Up** (65mm height) and **Sit Down** (0mm height).
* **Square:** Toggle **Balance Mode** On/Off.
* **D-Pad Up / Down:** Manually adjust **Body Height** (incremental).
* **D-Pad Left / Right:** Adjust **Gait Speed** (Left = Slower, Right = Faster).
* **L1:** Enter **Translate Mode** (Shift body).
* **L2:** Enter **Rotate Mode** (Tilt body).
* **Circle:** Enter **Single Leg Mode**.
* **Cross:** Enter **GP Player Mode** (Run pre-recorded sequences).

### Walk Mode (Default)
Active immediately after pressing Start.
* **Left Stick:** Walk Forward/Back and Strafe Left/Right.
* **Right Stick:** Rotate (Turn) Left/Right.
* **Select:** Cycle through the **5 Gaits** (Ripple, Tripod, Triple Tripod, etc.).
* **R1:** Toggle **Double Leg Lift Height** (robot steps higher).
* **R2:** Toggle **Double Travel Length** (robot takes longer strides).
* **R3 (Right Stick Click):** Toggle **Walk Method** (Swaps walk/rotate between sticks).

### Translate Mode (Hold L1)
Shifts the body's position relative to its feet.
* **Left Stick:** Shift body X (Side-to-Side) and Z (Forward-Backward).
* **Right Stick:** Shift body Y (Up-Down) and Rotate body Y (Yaw).

### Rotate Mode (Hold L2)
Tilts the body in place.
* **Left Stick:** Pitch (Tilt Forward/Back) and Roll (Tilt Side-to-Side).
* **Right Stick:** Rotate body Y (Yaw).

### Single Leg Mode (Circle)
Manually controls an individual leg.
* **Select:** Cycle active leg (RR, RM, RF, LR, LM, LF).
* **Left Stick:** Move the foot in X (Side-to-Side) and Z (Forward-Backward).
* **Right Stick:** Move the foot in Y (Up-Down).
* **R2:** **Hold/Lock** the leg in its current position.

### GP Player Mode (Cross)
Plays back sequences stored on the SSC-32.
* **Select:** Cycle through available Sequences (0–5).
* **R2:** **Start** the selected sequence.

### Safety Fallback
* **Connection Loss:** If the PS2 receiver loses connection for more than ~100ms, the robot will automatically execute an emergency sit-down and deactivate servos.

## Modernization & Refactoring Summary
This project recently underwent a complete architectural overhaul to transition from a monolithic, procedural structure into a clean, modular, and safe Object-Oriented architecture, without sacrificing the strict performance requirements of the 8-bit AVR hardware.

### Key Achievements
1. **Architecture:**
   - **`Leg` Class:** Encapsulates IK math, FK math, and coordinate state for each individual leg.
   - **`Hexapod` Class:** Manages robot-wide coordination, including Gaits, Balance, and Timing.
   - **Hardware Abstraction:** `InputController` and `ServoDriver` are now abstract "contracts," allowing the core mathematical engine to operate completely independently of the specific hardware peripherals.
2. **Safety & Reliability:**
   - **Regression Suite:** A full PC-based simulation and testing environment (`test/build.bat`) that verifies IK math, gait trajectories, PS2 input state machine transitions, and exact SSC-32 bitstream formatting against a "Golden Master" baseline.
   - **Timing Stability:** Implemented a fixed 50Hz (20ms) control loop and resolved historical blocking bugs to guarantee zero dropped packets or PS2 timeouts.
3. **Modernization:**
   - **Code Cleanup:** Localized global variables, extracted heavy `PROGMEM` lookup tables into `Hex_Tables.h`, and implemented a clean build pipeline.
   - **Strict Math Parity:** Ensured every bit of the original scaled integer math was preserved to maintain maximum performance on the ATmega328P.

### Refactored File Map
- **`BotBoarduino_CH3R_PS2.ino`**: High-level setup and main control loop.
- **`Hexapod.h / .cpp`**: The robot "brain" (Gait and Balance coordination).
- **`Leg.h / .cpp`**: The robotic limb "muscles" (Kinematics).
- **`InputController.h` / `PS2_controller.cpp`**: User interface layer.
- **`ServoDriver.h` / `phoenix_driver_ssc32.cpp`**: Hardware output layer.
- **`ControlState.h`**: Shared data structures (`COORD3D`, `INCONTROLSTATE`).
- **`Hex_Tables.h`**: Static math lookup tables and hardware configuration arrays.
- **`Hex_Cfg.h` / `Hex_Globals.h`**: Project configuration flags and external linkages.

## Testing
To verify the integrity of the math and logic engines on a PC, run the regression suite:
```bat
.\test\build.bat
```
This compiles the code using a mocked hardware environment and asserts 100% parity against saved Golden Master snapshots.
