# AGENTS.md

## Workspace Scope

This workspace contains two STM32 embedded projects:

1. `shiyanji_H7_250228_V1.19.8 (6) (cs5552 is ok)`
   - The main project.
   - STM32H750VBTx firmware for a materials testing machine.
   - Contains the real business logic: control loop, sensor handling, Ethernet protocol, persistence, and RTOS tasks.
2. `stm32h743vgt6`
   - A separate STM32H743VGTx CubeMX/Keil project skeleton.
   - Looks like a stripped-down scaffold or migration/bring-up project, not the main application.

There is no workspace-level submodule configuration in `.gitmodules` at the time of inspection.

## Which Project Matters Most

If you are trying to understand or modify the actual machine behavior, start in:

- `shiyanji_H7_250228_V1.19.8 (6) (cs5552 is ok)`

The `stm32h743vgt6` directory currently contains generated CubeMX-style startup code and empty RTOS task bodies. It does not contain the application logic found in the H750 project.

## Main Project Summary

Project root:

- `shiyanji_H7_250228_V1.19.8 (6) (cs5552 is ok)`

Target and toolchain:

- MCU: `STM32H750VBTx`
- IDE/build system: `MDK-ARM / Keil uVision`
- Project file: `MDK-ARM/shiyanji_H7.uvprojx`
- Keil toolchain setting: `ARM-ADS`
- `uAC6 = 0`, so this project is configured for classic ARMCC-style build, not ArmClang/AC6
- Output directory: `MDK-ARM/shiyanji_H7/`
- Output name: `shiyanji_H7`
- C/C++ mode:
  - Top-level project is C-oriented (`UseCPPCompiler=0`)
  - Do not assume a GCC/CMake workflow exists

Firmware identity:

- `Core/Src/main.c` defines `FIRMWAREVERSION` as `1.16.7`
- The folder name includes `V1.19.8`
- Treat the directory version and compiled firmware version as potentially inconsistent until verified on hardware or in release notes

## High-Level Function

The H750 project is a FreeRTOS-based embedded control application for a testing machine. The codebase combines:

- Closed-loop servo control
- Position/load/extensometer measurement processing
- Host communication over W5500 Ethernet
- RS485 / Modbus support
- Parameter persistence in MRAM and EEPROM
- Sensor calibration and mapping

The control modes visible in `Host/gParameter.h` include:

- `POS_MODE`
- `LOAD_MODE`
- `EXTEN_MODE`

## Directory Map

Within the main H750 project:

- `Core/`
  - CubeMX-generated startup, HAL init, RTOS integration, interrupt glue
  - `Core/Src/main.c` is the application entry
  - `Core/Src/freertos.c` creates tasks, mutexes, queue, and semaphore
- `Host/`
  - Main application layer
  - Key files:
    - `control.c`: control loop and command execution
    - `communicate.c`: host protocol parsing/dispatch
    - `in_out.c`: acquisition mapping, engineering-unit conversion, output mapping
    - `sensor.c`: sensor calibration, zero code, sync from stored params
    - `gParameter.c/.h`: global state, mode/state enums, shared structs
    - `mram_manage.c`, `Eeprom_manage.c`: persistent parameter handling
    - `posGenerator.c`, `trackplaning.c`: trajectory/motion generation
- `Hardware/`
  - Board-facing drivers and helpers
  - Includes `CS5552`, `DAC8831`, `RS485`, `Servo_driver`, `Encoder`, `DI`, `DO`, `EEPROM`, filters, FIFO helpers
- `Ethenet/`
  - W5500 transport and Ethernet-side runtime
  - Note the directory is spelled `Ethenet`, not `Ethernet`
- `Internet/`
  - Protocol stack components and examples/integration folders:
    - `DHCP`, `DNS`, `FTPClient`, `FTPServer`, `httpServer`, `MQTT`, `SNMP`, `SNTP`, `TFTP`
- `ModbusLib/`
  - Contains `MODBUS.md`
- `Easylogger/`
  - Logging library integration
- `ADRC/`, `PID/`, `Regression/`
  - Control and math/algorithm support
- `Drivers/`, `Middlewares/`
  - STM32 HAL/CMSIS/FreeRTOS and vendor middleware
- `MDK-ARM/`
  - Keil project and build outputs

Other noteworthy files:

- `user.c`, `user.h`
  - Local CS5552 support code and low-level timing/SPI helpers
- `CS5552_DATAFLOW.md`
  - The clearest project-specific note about measurement data flow
- `CS5552.pdf`
  - ADC datasheet/reference material
- `CLAUDE.md`
  - A concise, readable project summary inside the project root
- `AGENTS.md`
  - There is already a project-local file with this name, but the copy inspected here appears to have encoding damage and should not be treated as the canonical readable reference

## Boot And Runtime Flow

From `Core/Src/main.c`:

1. HAL/CubeMX peripheral init runs first.
2. `hardwareInit()` initializes board-facing pieces such as:
   - DAC chip select defaults
   - MRAM chip select defaults
   - servo pulse/direction setup
   - Modbus / RS485
   - CS5552 compatibility init
   - FIFO
   - encoder
3. `softwareInit()` initializes:
   - sensor calibration callbacks
   - work state and mode defaults
   - global/default parameters
   - MRAM-backed state
   - Ethernet configuration
   - sensor data sync
   - PID/ADRC-related state
   - DI function map
4. `MX_FREERTOS_Init()` creates RTOS objects.
5. `osKernelStart()` starts the scheduler.

## RTOS Structure

The code uses:

- FreeRTOS
- CMSIS-RTOS v1 style APIs (`cmsis_os.h`, `osThreadDef`, `osSemaphoreCreate`, etc.)

Visible tasks in `Core/Src/freertos.c`:

- `defaultTask`
- `loopTask`
- `watchdogTask`
- `comTask`
- `waveTask`
- `ethernetTask`
- `cdatatranstask`
- `eepromTask`

Visible synchronization objects:

- Mutexes:
  - `Controller_statusMutex`
  - `pgMutex`
  - `moveparaMutex`
- Semaphore:
  - `cs5552Sem`
- Queue:
  - `lpusTANQueue`

Observed behavior notes:

- `loopTask` is the main real-time application loop.
- `defaultTask` currently prints filtered voltage/debug counters every 100 ms / 1 s cadence.
- Comments in `Host/in_out.c` indicate CS5552 sampling responsibility was moved out of the older path and into another runtime path.

## Data And Control Flow

The most useful mental model is:

1. Raw hardware data is read from encoder / DI / DB9 / CS5552-related sources.
2. `Host/in_out.c`
   - maps raw IO
   - converts sensor codes into engineering values
   - applies tare, calibration, filtering, and derived calculations
3. `Host/control.c`
   - updates command state
   - updates closed-loop parameters
   - chooses movement/control mode
   - runs the control algorithm
4. Output mapping/control sends commands to:
   - servo pulse output
   - DAC8831 analog output
   - direction / SON / DO / relay lines

The dedicated project note `CS5552_DATAFLOW.md` shows this flow in more detail for:

- `force.filter`
- `strain2.filter`
- the broader `inputGetValue() -> inputMapping() -> controlLoop() -> outputControl()` path

## Communication Architecture

### Ethernet

`Ethenet/ETHw5500.c` shows:

- W5500 network transport
- static network defaults compiled into the code before parameter sync:
  - local IP default `192.168.0.11`
  - server IP default `192.168.0.10`
  - server port default `5001`
- runtime network values are then loaded from `ethConfig`

The Ethernet path appears to be:

- W5500 transport in `Ethenet/ETHw5500.c`
- protocol framing/dispatch in `Host/communicate.c` and `Host/EthProtocol.c`

### Host Command Handling

`Host/communicate.c` contains a large command dispatcher for protocol opcodes such as:

- link control
- enable/disable
- transmit settings
- parameter read/write
- PID/speed/feedforward config
- sensor data read/write
- movement commands
- halt / trigger / block / cycle commands

This is a compatibility-sensitive area. Treat packet formats and parameter layouts as contracts.

### RS485 / Modbus

The project initializes:

- `modbusInit()`
- `RS485_init(&huart2)`

So UART2/RS485 and Modbus are part of the board runtime, not just dead code.

## Sensor And Measurement Notes

The newer work in this tree is strongly centered on `CS5552`.

Evidence:

- `Hardware/CS5552.c`, `Hardware/CS5552.h`
- `user.c`, `user.h`
- `CS5552_DATAFLOW.md`
- comments in `main.c`, `freertos.c`, and `in_out.c`

Important observations:

- The old ADC path has comments marking parts as retired or legacy.
- `Host/in_out.c` currently keeps `strain1.code = 0` with an explicit comment saying chip0.ch0 is not used in the current CS5552 migration.
- `Host/in_out.c` maps:
  - force from `CS5552_COMPAT_CHANNEL_FORCE`
  - strain2 from `CS5552_COMPAT_CHANNEL_STRAIN2`
- `sensor.c` contains the calibration pipeline and zero-code handling for pose, big deformation, ext1, ext2, and load channels.

## Persistence And Global State

The project relies heavily on shared global state.

Most important hub:

- `Host/gParameter.h`

It defines or anchors:

- work mode/state enums
- emergency flags
- limit flags
- control mode selection
- sensor-related shared state
- application-layer shared structs

Persistence-related logic spans:

- `Host/gParameter.c`
- `Host/mram_manage.c`
- `Host/Eeprom_manage.c`
- `sensor.c`

When adding or changing persistent parameters, update all related storage and synchronization paths together.

## Build Reality

Practical build facts observed in this workspace:

- The real build entry is the Keil project file, not a workspace-level script.
- No app-level `build.bat`, `build.ps1`, `Makefile`, or top-level `CMakeLists.txt` was found for either project.
- `Drivers/CMSIS/...` contains vendor/example `CMakeLists.txt` files, but those are not the application build system.

For the main project, use:

- `shiyanji_H7_250228_V1.19.8 (6) (cs5552 is ok)/MDK-ARM/shiyanji_H7.uvprojx`

For the secondary skeleton project, use:

- `stm32h743vgt6/MDK-ARM/stm32h743vgt6.uvprojx`

## Secondary Project: `stm32h743vgt6`

This directory appears to be a fresh or simplified CubeMX-generated project:

- MCU: `STM32H743VGTx`
- Keil target: `stm32h743vgt6`
- Toolchain: `ARM-ADS`
- Output dir: `MDK-ARM/stm32h743vgt6/`

Characteristics:

- Standard CubeMX layout: `Src/`, `Inc/`, `Drivers/`, `Middlewares/`, `MDK-ARM/`
- `Src/main.c` performs peripheral init and starts FreeRTOS
- `Src/freertos.c` creates the same family of tasks/mutexes/queue/semaphore names as the main project
- task bodies are still placeholder `osDelay(1)` loops

Interpretation:

- It likely serves as a bring-up base, migration attempt, or clean reference project
- It is not the source of the machine-specific business logic found in the H750 project

## Recommended Reading Order

For the main project, read in this order:

1. `Core/Src/main.c`
2. `Core/Src/freertos.c`
3. `Host/control.c`
4. `Host/communicate.c`
5. `Host/in_out.c`
6. `Host/sensor.c`
7. `Host/gParameter.h`
8. `Ethenet/ETHw5500.c`
9. `Hardware/CS5552.c`
10. `CS5552_DATAFLOW.md`

If the task is specifically about persistence/configuration, also read:

- `Host/gParameter.c`
- `Host/mram_manage.c`
- `Host/Eeprom_manage.c`

## Working Rules For Future Edits

These are the safest assumptions for this workspace:

- Treat `Host/` as the main application layer.
- Minimize changes in `Drivers/`, `Middlewares/`, and vendor CMSIS code.
- Prefer C, not C++.
- Do not assume GCC/Clang compatibility if the code relies on ARMCC behavior.
- Be careful with protocol compatibility in `communicate.c` and related framing code.
- Be careful with parameter ordering/layout in global structs and persistent storage.
- Keep ISR work lightweight and avoid introducing blocking behavior into interrupt context.
- Quote paths in scripts/commands because the main project folder contains spaces and parentheses.

## Useful Gotchas

- The main project directory name contains spaces and parentheses.
- `Ethenet` is intentionally misspelled in the tree; do not "fix" imports casually.
- There are several `.bak` files in `Host/`, which suggests manual backup snapshots are part of the developer workflow.
- The existing project-local `AGENTS.md` appears to have encoding issues; the project-local `CLAUDE.md` is more readable.
- The root workspace now has this `AGENTS.md`, which should be treated as the clean summary for the whole folder.
