# BotBoarduino CH3R PS2 (Lynxmotion Phoenix Port)

## Project Overview
This repository contains a modernized, Object-Oriented C++ implementation of the classic Lynxmotion Phoenix hexapod codebase. It is specifically configured for a circular hexapod (CH3R) utilizing a **BotBoarduino** (or Arduino Pro Mini / ATmega328P), an **SSC-32 Servo Controller**, and a **PS2 Wireless Controller**.

The codebase leverages highly optimized, 16-bit scaled integer math and `PROGMEM` lookup tables to execute complex 3D Inverse Kinematics (IK), 3D Forward Kinematics (FK), and gait generation strictly within a 50Hz (20ms) control loop on an 8-bit microcontroller, completely avoiding slow floating-point overhead.

## Hardware Requirements
* **Microcontroller:** Lynxmotion BotBoarduino (ATmega328P, 16MHz)
* **Servo Controller:** Lynxmotion SSC-32 (configured for Binary Mode `cSSC_BINARYMODE`)
* **Input:** PS2 Wireless Controller
* **Robot Chassis:** CH3R (Circular Hexapod 3-DOF, with optional 4-DOF support)

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
