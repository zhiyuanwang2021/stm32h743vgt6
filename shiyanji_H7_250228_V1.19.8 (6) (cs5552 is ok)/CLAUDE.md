# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

See `AGENTS.md` for the full project handbook — this file distills the essentials for Claude Code sessions.

## Project Overview

STM32H750VBTx embedded firmware for a materials testing machine (试验机). Closed-loop servo control with custom host protocol over W5500 Ethernet. FreeRTOS + CMSIS-RTOS v1 via `cmsis_os.h`.

## Build

- **IDE**: MDK-ARM (Keil uVision), toolchain ARMCC
- **Project file**: `MDK-ARM/shiyanji_H7.uvprojx`
- **Target name**: `shiyanji_H7`
- **Output**: `MDK-ARM/shiyanji_H7/` (hex + bin via fromelf post-processing)
- **C standard**: C99 (ARMCC-compatible)

No CLI build scripts exist. To build, open the `.uvprojx` in Keil and rebuild. When adding new `.c` files, also add them to the project groups in `uvprojx`.

## Architecture (layered)

| Layer | Directory | Role |
|-------|-----------|------|
| Application / Business | `Host/` | Control loop, protocol dispatch, sampling & calibration, global params, sensor mgmt, MRAM/EEPROM persistence |
| Algorithms | `PID/`, `ADRC/`, `Regression/` | Control and system identification |
| Board / Hardware | `Hardware/` | DI/DO, RS485, encoder, MRAM, servo driver, DAC (DAC8831), ADC legacy driver/code and current acquisition support |
| Network transport | `Ethenet/` | W5500 Ethernet driver (note: directory is `Ethenet`, not `Ethernet`) |
| Protocol stacks | `Internet/`, `ModbusLib/` | MQTT, SNMP, HTTP, FTP, DHCP, DNS, SNTP, TFTP, Modbus |
| Platform / BSP | `Core/`, `Drivers/`, `Middlewares/` | HAL, CMSIS, FreeRTOS — **minimize changes here** |
| Logging | `Easylogger/` | Log library with FreeRTOS adapter |

### Key files to read first

`Core/Src/main.c` → `Core/Src/freertos.c` → `Host/control.c` → `Host/communicate.c` → `Host/in_out.c` → `Host/gParameter.h` → `Host/sensor.c` → `Ethenet/ETHw5500.c`

### Primary data flow

Sensors / ADC front-end → `Hardware/CS5552.c` and related board drivers → `Host/in_out.c` (sampling, unit conversion) → `Host/control.c` (closed-loop PID/ADRC, trajectory generation in `posGenerator.c`, path planning in `trackplaning.c`) → servo output via `Hardware/DAC8831.c` / `Hardware/Servo_driver.c`

Host commands arrive via `Ethenet/ETHw5500.c` → `Host/communicate.c` (command dispatch per `EthProtocol.c`) → control state changes or parameter reads

## Critical constraints

- **C only**. Do not rewrite C as C++. ARMCC/C99 compatibility required. No GCC extensions.
- **Do not modify** `Drivers/`, `Middlewares/`, `CMSIS/` unless absolutely necessary.
- **ISRs must stay lightweight** — no blocking ops in interrupt handlers.
- **Global parameter layout is a compatibility contract** — do not reorder `gParameter` structs.
- When adding persistent parameters, check `gParameter.c`, `mram_manage.c`, and `Eeprom_manage.c` together.
- When touching protocol code, verify wire-format compatibility.
- **No formatter tooling** — follow prevailing file style, don't reformat whole files.
- Paths must be quoted in shell commands (the workspace path contains spaces and parentheses).

## Environment

Windows + PowerShell. File encoding: source is UTF-8. Line endings: CRLF (keep existing style per file).
