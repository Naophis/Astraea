# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the Astraea/AstraeaNova micromouse robot project using ESP32-S3 microcontroller. The project implements a dual-core FreeRTOS system for autonomous maze navigation, with separate cores handling real-time sensing and high-level planning tasks.

## Build Commands

### Core Build Commands
- `idf.py build` - Build the entire project
- `./compile.sh` - Convenience script that calls `idf.py build`
- `./flash.sh` - Flash firmware to ESP32-S3 device via USB
- `./erase.sh` - Erase flash memory
- `./console.sh` - Open serial console for debugging
- `./trace.sh` - Analyze stack traces using addr2line

### Development Workflow
1. `idf.py build` - Compile the project
2. `./flash.sh` - Flash to device
3. `./console.sh` - Monitor output

## Architecture

### Dual-Core System Design
The system utilizes ESP32-S3's dual-core architecture:
- **Core 0**: Real-time sensing and control tasks
- **Core 1**: Planning and navigation tasks

### Core Components

#### Main Source Structure
- `main/main.cpp` - Application entry point
- `main/include/` - Header files for all modules
- `main/task/` - Task implementation files
- `main/search/` - Maze solving algorithms (Adachi, logic)

#### Key Tasks
- **SensingTask**: Handles sensors (IMU, encoders, wall sensors) at 1kHz
- **MainTask**: Core control loop and state management
- **PlanningTask**: Path planning and maze solving
- **LoggingTask**: Data logging to SD card/flash

#### Sensor Integration
- **IMU**: ASM330LHH (newer) or LSM6DSR/ICM20689 (legacy)
- **Encoders**: AS5147P 14-bit magnetic encoders
- **Wall Sensors**: LED + phototransistor pairs with BCR421UFD current regulation

#### Generated Code Integration
- `gen_code_mpc/` - MATLAB-generated MPC controller
- `gen_code_pid/` - PID controller implementations
- `gen_code_conv_single2half/` - Float to half-precision conversion

### Hardware Configuration
- **MCU**: ESP32-S3-PICO-1 with 2MB PSRAM
- **Motor Driver**: MP6551 with low internal resistance
- **Power**: LXDC55 DC-DC converter + MIC5219 3.3V LDO
- **Wall Sensors**: OSI5FU3A11C LEDs + LTR-4206E phototransistors

## Key Files to Understand

### Core Headers
- `main/include/defines.hpp` - Global constants, GPIO definitions, hardware config
- `main/include/structs.hpp` - Data structures for sensor data, control states
- `main/include/enums.hpp` - System state enumerations
- `main/include/base_task.hpp` - Base class for all tasks

### Algorithm Files
- `main/include/maze_solver.hpp` - Maze representation and solving
- `main/search/adachi.cpp` - Adachi maze solving algorithm
- `main/task/motion_planning.cpp` - Trajectory generation
- `main/task/kalman_filter.cpp` - State estimation

## Development Notes

### Performance Optimizations
- Critical functions are placed in IRAM for faster execution
- Custom ESP-IDF modifications for SPI and ADC performance
- FreeRTOS queue replaced with notify for inter-task communication
- Sensor sampling optimized to reduce system load

### Testing and Validation
- Extensive logging system for performance analysis
- Parameter tuning tools in `tools/param_tuner/`
- MATLAB simulation environment in `tools/matlab_ws/`
- CSV data analysis scripts throughout various csv directories

### Known Issues
- ESP32-S3 has undocumented pins that default to HIGH on boot (GPIO20, GPIO26, GPIO39)
- FreeRTOS stability issues with high PWM duty cycles - resolved by removing crystal oscillator capacitors
- Timing critical code must be carefully placed in IRAM vs Flash

## Tools Directory
- `tools/param_tuner/` - Parameter tuning interface
- `tools/matlab_ws/` - MATLAB simulation and code generation
- `tools/mm-maze-viewer/` - VS Code extension for maze visualization
- `tools/rosws/` - ROS workspace for simulation

## Calibration
- `tools/param_tuner/logs/*.csv` - csvｗのログファイルを格納。plotjugglerを使って可視化します。
