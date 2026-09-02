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

**Flashing the firmware:**
1. Hold down the **BOOTSEL** button while connecting the Pico / BTT SKR Pico to your computer via USB.
2. Mount the board as mass storage (`RPI-RP2`).
3. Copy or drag-and-drop `scara-base.uf2` into the drive. The device will automatically reboot and run the firmware.

### Usage

The firmware exposes a serial interface over USB CDC (`115200` baud) for real-time control, configuration, and trajectory execution.

**Common Commands:**

```text
# System state & control
<CMD:STATUS>                             # Query current state and endstop sensor status
<CMD:ENABLE>                             # Enable stepper motor drivers
<CMD:DISABLE>                            # Disable stepper motor drivers
<CMD:HOME>                               # Trigger homing sequence
<CMD:ESTOP>                              # Immediate emergency stop

# Motion & Waypoints (linear interpolation with analytical IK)
<pt#X#Y#Z#PHI#SPEED#end>                 # Waypoint: X, Y, Z (mm), PHI (tool angle deg), SPEED (mm/s)
<CMD:GETPOS>                             # Read current Cartesian coordinates and tool pose

# Configuration & Flash persistence
<CMD:GET_CONFIG>                         # Read runtime configuration (L1, L2, limits, speeds)
<CMD:SAVE_CONFIG>                        # Save current configuration into on-board Flash
<CMD:RESET_CONFIG>                       # Restore factory default configuration
```

### Dependencies

**scara** firmware requires the following toolchain and libraries:

* **Pico SDK**: [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) (>= 1.5.0)
* **Compiler**: `arm-none-eabi-gcc` and `arm-none-eabi-newlib`
* **Build Tools**: `CMake` (>= 3.12) and `Make` or `Ninja`
* **Target Hardware**: Raspberry Pi Pico (RP2040) or BTT SKR Pico V1.0 board

### Project structure

**scara** codebase is organized into hardware and dual-core software modules:

```text
scara/
├── docs/                 # Documentation, specifications, and assets (logo)
├── hw/                   # Hardware design, CAD files, and schematics
└── sw/
    └── scara_base/       # RP2040 dual-core C11/C++17 firmware
        ├── src/
        │   ├── command/     # Packet parser, command dispatcher & ring buffer queue
        │   ├── config/      # Runtime configuration and persistent Flash storage
        │   ├── io/          # GPIO mapping, limit switches/endstops and LED status
        │   ├── kinematics/  # Analytical Forward & Inverse Kinematics (L1, L2)
        │   ├── motion/      # Cartesian trajectory planner and interpolation
        │   ├── stepper/     # Multi-axis stepper driver utilizing RP2040 PIO
        │   └── main.c       # Dual-core firmware entry point
        └── CMakeLists.txt   # CMake configuration for Pico SDK
```

### Docs

[![Documentation Status](https://readthedocs.org/projects/scara/badge/?version=latest)](https://scara.readthedocs.io/projects/scara/en/latest/?badge=latest)

More documentation and info at
* [https://scara.readthedocs.io/en/latest/](https://scara.readthedocs.io/en/latest/)

### Copyright and licence

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0) [![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

Copyright (C) 2020 by [dof2bot.github.io/scara](https://dof2bot.github.io/scara)
