<img align="right" src="https://github.com/dof2bot/scara/blob/master/docs/scara_logo.png" width="25%">

**scara** is an open-source SCARA robotic arm manipulator.

[scara](https://en.wikipedia.org/wiki/SCARA) is developed in C code.

The README is used to introduce the tool and provide instructions on
how to install the tool, any machine dependencies it may have and any
other information that should be provided before the tool is installed.

<!-- START doctoc generated TOC please keep comment here to allow auto update -->
<!-- DON'T EDIT THIS SECTION, INSTEAD RE-RUN doctoc TO UPDATE -->
**Table of Contents**

- [Installation](#installation)
- [Usage](#usage)
- [Dependencies](#dependencies)
- [Project structure](#project-structure)
- [Docs](#docs)
- [Copyright and licence](#copyright-and-licence)

<!-- END doctoc generated TOC please keep comment here to allow auto update -->

### Installation

Clone the repository and build the firmware for Raspberry Pi Pico (RP2040):

```bash
# Clone the repository
git clone https://github.com/dof2bot/scara.git
cd scara/sw/scara_base

# Set the Pico SDK path (if not set globally)
export PICO_SDK_PATH=/path/to/pico-sdk

# Create build directory and compile
mkdir build && cd build
cmake ..
make -j$(nproc)
```

Alternatively, use the provided helper scripts:

```bash
# Setup udev rules for non-root picotool and USB serial access (optional)
./scripts/setup_udev.sh

# Build firmware
./scripts/flash.sh build

# Build and flash automatically to connected Pico via picotool or mass-storage
./scripts/flash.sh build-and-flash
```

**Flashing the firmware manually:**
1. Hold down the **BOOTSEL** button while connecting the Pico / BTT SKR Pico to your computer via USB.
2. Mount the board as mass storage (`RPI-RP2`).
3. Copy or drag-and-drop `build/src/scara-base.uf2` into the drive. The device will automatically reboot and run the firmware.

### Usage

The firmware exposes a serial interface over USB CDC (`115200` baud) for real-time control, configuration, and trajectory execution.

The firmware runs a **dual-core architecture**:
* **Core 0**: Handles non-blocking USB CDC serial communication, ASCII protocol parsing, command dispatching, safety monitoring, and heartbeat LED.
* **Core 1**: Dedicated real-time motion worker handling the waypoint trajectory queue, analytical Inverse Kinematics (IK), trapezoidal velocity profiling, and multi-axis stepper pulse generation.

#### Serial Command Reference

```text
# System State & Control
<CMD:STATUS>                             # Query current state (IDLE, MOVING, HOMING, HOLD, ESTOP) and endstops
<CMD:ENABLE>                             # Energize and enable stepper motor drivers
<CMD:DISABLE>                            # De-energize and disable stepper motor drivers
<CMD:HOME>                               # Trigger multi-axis coordinated homing sequence
<CMD:ESTOP>                              # Immediate emergency stop and motion abort
<CMD:HOLD>                               # Feed hold / pause trajectory execution (alias: <CMD:PAUSE>)
<CMD:RESUME>                             # Resume paused trajectory execution

# Motion & Positioning
<pt#X#Y#Z#PHI#SPEED#end>                 # Push Cartesian waypoint: X, Y, Z (mm), PHI (tool angle deg), SPEED (mm/s)
<CMD:GETPOS>                             # Read current Cartesian coordinates and tool orientation
<CMD:SETPOS#X#Y#Z#PHI#end>               # Set/override current Cartesian coordinate frame origin
<CMD:JOG#axis#step>                      # Manual relative jog (axis: 'X', 'Y', 'Z', or 'P' for PHI; step: mm or deg)

# Kinematics & Arm Configuration
<CMD:SET_ELBOW#LEFT>                     # Select Left-arm (elbow left) inverse kinematics configuration
<CMD:SET_ELBOW#RIGHT>                    # Select Right-arm (elbow right) inverse kinematics configuration
<CMD:GET_ELBOW>                          # Read current elbow configuration

# End-Effector Tool & Real-Time Control
<CMD:PUMP#1>                             # Turn ON vacuum/pump actuator
<CMD:PUMP#0>                             # Turn OFF vacuum/pump actuator
<CMD:VALVE#1>                            # Open air release valve
<CMD:VALVE#0>                            # Close air release valve
<CMD:WAIT#ms>                            # Dwell delay pause for ms milliseconds
<CMD:OVERRIDE#percent>                   # Dynamic feedrate velocity scaling (10 to 200 %)

# Runtime Configuration & Flash Persistence
<CMD:GET_CONFIG>                         # Read runtime configuration (link lengths, limits, dynamics, speeds)
<CMD:SET_CONFIG#L1=..#L2=..#Z_MIN=..#Z_MAX=..#MIN_SPEED=..#MAX_SPEED=..> # Configure geometry and speed bounds
<CMD:SET_DYNAMICS#ACCEL=..#MAX_ACCEL=..#DEF_SPD=..>                      # Configure motion dynamics
<CMD:SET_HOMING#OFF_J1=..#OFF_J2=..#RATE=..>                             # Configure homing offsets and rate
<CMD:SET_LIMITS#J1_MIN=..#J1_MAX=..#J2_MIN=..#J2_MAX=..>                 # Configure joint angle limits (rad)
<CMD:SET_STEPS#GR_J1=..#GR_J2=..#GR_J4=..#LEAD_Z=..>                     # Configure gear ratios and Z lead
<CMD:SAVE_CONFIG>                        # Save active configuration into on-board Flash (with CRC32 check)
<CMD:RESET_CONFIG>                       # Restore factory default configuration parameters
```

#### Serial Response Reference

```text
<RESP:ACK#QUEUE=n>                       # Waypoint accepted; n buffer slots remaining
<RESP:MOVE_START#X=..#Y=..#Z=..#PHI=..>  # Physical waypoint execution initiated
<RESP:MOVE_DONE#X=..#Y=..#Z=..#PHI=..>   # Physical waypoint execution completed
<RESP:ACK#WAIT_DONE#MS=ms>               # Hardware dwell delay completed
<RESP:ACK#PUMP_ON> / <RESP:ACK#PUMP_OFF> # Pump actuation acknowledged
<RESP:ACK#VALVE_ON> / <RESP:ACK#VALVE_OFF> # Valve actuation acknowledged
<RESP:ACK#OVERRIDE=percent>              # Velocity override scaling updated
<RESP:HOMED_SUCCESS#X=..#Y=..#Z=..#PHI=..> # Homing cycle completed successfully
<RESP:NACK_BUFFER_FULL>                  # Input ring buffer saturated
<RESP:NACK_ESTOP_ACTIVE>                 # Emergency stop active; commands rejected
```

### Dependencies

**scara** firmware requires the following toolchain and libraries:

* **Pico SDK**: [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) (>= 1.5.0)
* **Compiler**: `arm-none-eabi-gcc` and `arm-none-eabi-newlib`
* **Build Tools**: `CMake` (>= 3.12) and `Make` or `Ninja`
* **Flashing Tools**: `picotool` (optional, for automated flashing and reboot)
* **Target Hardware**: Raspberry Pi Pico (RP2040) or BTT SKR Pico V1.0 board

### Project structure

**scara** codebase is organized into hardware designs, documentation, and dual-core firmware modules:

```text
scara/
├── docs/                 # Documentation, specifications, and assets (logo)
├── hw/                   # Hardware design, CAD files, and schematics
└── sw/
    └── scara_base/       # RP2040 dual-core C11 firmware
        ├── scripts/      # Automation utilities (flash.sh, setup_udev.sh)
        ├── src/
        │   ├── command/     # Serial packet parser, dispatcher, and trajectory queue
        │   │   └── handlers/# Modular handlers (cmd_system, cmd_motion, cmd_config, cmd_tool)
        │   ├── config/      # Runtime configuration, defaults, and Flash persistent storage
        │   ├── homing/      # Multi-axis coordinated homing controller & state machine
        │   ├── io/          # GPIO mapping, endstop debouncing, tool outputs & status LED
        │   ├── kinematics/  # Analytical IK/FK solver & safety guard workspace bounds
        │   ├── motion/      # Motion planner, trapezoidal velocity profiling & trajectory chunking
        │   ├── stepper/     # Multi-axis stepper driver & timer/PIO pulse generation
        │   └── main.c       # Dual-core firmware entry point (Core 0: Comm, Core 1: Motion)
        └── CMakeLists.txt   # CMake configuration for Pico SDK
```

### Docs

[![Documentation Status](https://readthedocs.org/projects/scara/badge/?version=latest)](https://scara.readthedocs.io/projects/scara/en/latest/?badge=latest)

More documentation and info at
* [https://scara.readthedocs.io/en/latest/](https://scara.readthedocs.io/en/latest/)

### Copyright and licence

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Copyright (C) 2020 - 2026 by [dof2bot.github.io/scara](https://dof2bot.github.io/scara)
