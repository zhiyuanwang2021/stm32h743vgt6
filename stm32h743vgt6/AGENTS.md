# AGENTS.md

## Scope

This file documents the current folder:

- `stm32h743vgt6`

This directory is an STM32H743VGTx CubeMX/Keil project skeleton. It is not the main application firmware for the testing machine. The machine-specific business logic lives in the sibling project:

- `shiyanji_H7_250228_V1.19.8 (6) (cs5552 is ok)`

If the task is about real control behavior, host protocol, sensor calibration, MRAM persistence, CS5552 data flow, or Ethernet command handling, start in the H750 project instead of this folder.

## What This Project Appears To Be

This directory looks like a stripped-down bring-up or migration base for STM32H743VGTx:

- standard CubeMX-style generated structure
- Keil MDK-ARM project files
- HAL/CMSIS/FreeRTOS middleware layout
- peripheral init in `main.c`
- RTOS object creation in `freertos.c`
- placeholder task bodies instead of machine logic

Practical interpretation:

- use this tree as a hardware/RTOS scaffold, reference project, or porting base
- do not assume it contains the production behavior from the H750 codebase

## Target And Build Facts

Observed/project-level facts for this folder:

- MCU family: `STM32H743VGTx`
- IDE/build system: `MDK-ARM / Keil uVision`
- Project file: `MDK-ARM/stm32h743vgt6.uvprojx`
- Keil target/output name: `stm32h743vgt6`
- Toolchain family: `ARM-ADS`

Working assumption for future edits:

- prefer C, not C++
- treat the Keil project as the build truth
- do not assume GCC/CMake or a top-level script-based build exists

## Likely Directory Layout

This folder follows the familiar CubeMX/Keil pattern:

- `Src/`
  - generated startup flow and application entry
  - `main.c`
  - `freertos.c`
  - interrupt/peripheral glue
- `Inc/`
  - generated headers and config
- `Drivers/`
  - STM32 HAL, CMSIS, vendor code
- `Middlewares/`
  - FreeRTOS and related middleware
- `MDK-ARM/`
  - Keil project files and build output

Treat `Drivers/` and vendor middleware as generated or third-party code unless there is a strong reason to modify them.

## Runtime Shape

The runtime appears to be a standard CubeMX + FreeRTOS bootstrap:

1. HAL init and system clock setup
2. peripheral initialization from generated `MX_*` functions
3. RTOS object creation in `MX_FREERTOS_Init()`
4. scheduler start
5. task execution in the created threads

At the time of inspection summary, this project does not appear to contain the higher-level testing-machine workflow that exists in the H750 project.

## FreeRTOS Structure

This folder appears to create the same family of RTOS objects as the main project, but without the real application logic wired in yet.

Visible task names associated with this tree:

- `defaultTask`
- `loopTask`
- `watchdogTask`
- `comTask`
- `waveTask`
- `ethernetTask`
- `cdatatranstask`
- `eepromTask`

Visible synchronization objects associated with this tree:

- mutexes similar to:
  - `Controller_statusMutex`
  - `pgMutex`
  - `moveparaMutex`
- semaphore:
  - `cs5552Sem`
- queue:
  - `lpusTANQueue`

Important caveat:

- in this H743 project, task bodies are described as placeholder loops with `osDelay(1)` behavior
- do not mistake matching task names for a full port of the H750 application

## Relationship To The Main H750 Project

This folder is best understood as structurally similar to the main firmware, not functionally equivalent to it.

Things that exist in the H750 main project but should not be assumed to exist here in usable form:

- control loop business logic in `Host/control.c`
- host/Ethernet command dispatch in `Host/communicate.c`
- IO mapping and engineering conversion in `Host/in_out.c`
- sensor calibration/state sync in `Host/sensor.c`
- MRAM/EEPROM-backed parameter persistence
- CS5552-based acquisition integration
- W5500 transport runtime

If a change request mentions any of those subsystems, check whether the request actually targets the sibling H750 project.

## Best Reading Order For This Folder

When you need to understand this directory, read in this order:

1. `Src/main.c`
2. `Src/freertos.c`
3. `Inc/main.h`
4. `Inc/freertos.h`
5. `Src/stm32h7xx_it.c`
6. `Src/stm32h7xx_hal_msp.c`
7. `MDK-ARM/stm32h743vgt6.uvprojx`

Then inspect any generated peripheral init functions relevant to the task, such as UART, SPI, TIM, DMA, ETH, GPIO, or cache/MPU setup.

## Safe Working Rules

For future edits in this folder:

- keep changes scoped to application-owned files first
- prefer `Src/` and `Inc/` over editing HAL driver internals
- minimize changes in `Drivers/` and `Middlewares/`
- keep ISR work short and non-blocking
- do not add business assumptions copied from the H750 project unless you are intentionally porting them
- if you are porting logic from the H750 project, document which modules are source-of-truth and which interfaces must remain compatible

## Good Use Cases For This Folder

This project is a reasonable place for:

- STM32H743 bring-up
- clock/cache/MPU validation
- peripheral initialization experiments
- FreeRTOS object/task scaffolding
- early driver integration
- comparing CubeMX-generated startup against the H750 production tree
- staged migration work before pulling in full application logic

## Things To Verify Before Reusing It

Before treating this project as a serious port target, verify:

- clock tree and power configuration for H743
- MPU/cache setup
- interrupt priority configuration
- DMA and memory-region assumptions
- FreeRTOS heap/stack sizing
- linker/scatter configuration in Keil
- whether task names match the main project only cosmetically or by intended interface

## Bottom Line

The current folder is best treated as an H743 scaffold project with familiar RTOS naming, not as the authoritative machine-control firmware. It is useful as a clean reference and a possible migration base, but the real application behavior still belongs to the sibling H750 project unless future work explicitly ports that logic here.
