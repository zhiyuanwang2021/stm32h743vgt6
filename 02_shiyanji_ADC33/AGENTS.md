# AGENTS.md

## Project Summary

- This workspace is a STM32H750VBTx firmware project generated from STM32CubeMX and built with Keil MDK-ARM.
- The current application focus is dual-CS5552 acquisition over `SPI3`, simple sliding-average filtering, and debug/data output over `USART1`.
- The codebase is a snapshot folder, not a Git checkout in its current form.

## Directory Map

- `Core/Inc`, `Core/Src`: active application and CubeMX-generated firmware sources used by the Keil target.
- `Drivers/`: STM32 HAL and CMSIS vendor code. Treat as read-only unless the task explicitly requires vendor-layer changes.
- `MDK-ARM/`: Keil project files, startup, generated scatter/build artifacts, map/html size reports.
- `shiyanji_H7.ioc`: CubeMX source configuration.
- `.vscode/c_cpp_properties.json`: editor IntelliSense settings only, not the real build configuration.
- `CS5552.pdf`: chip datasheet/reference material for the ADC path.

## Actual Build Inputs

- Keil target name: `shiyanji_H7`.
- Device: `STM32H750VBTx`.
- Active preprocessor defines in Keil: `USE_PWR_LDO_SUPPLY`, `USE_HAL_DRIVER`, `STM32H750xx`.
- Active include roots in Keil:
  - `Core/Inc`
  - `Drivers/STM32H7xx_HAL_Driver/Inc`
  - `Drivers/STM32H7xx_HAL_Driver/Inc/Legacy`
  - `Drivers/CMSIS/Device/ST/STM32H7xx/Include`
  - `Drivers/CMSIS/Include`
- Keil compiles `Core/Src/*.c`, not the duplicate top-level `Core/*.c` files.
- The project uses the generated startup file `MDK-ARM/startup_stm32h750xx.s`.

## Duplicate File Warning

- `Core/user.c` and `Core/filter.c` exist alongside `Core/Src/user.c` and `Core/Src/filter.c`.
- Keil only references `Core/Src/user.c` and `Core/Src/filter.c`.
- Treat the top-level `Core/*.c` duplicates as likely historical copies unless the user explicitly says they are authoritative.

## MCU, Clock, and Memory Facts

- External HSE is configured at `8 MHz`.
- System clock is configured from PLL1 to `400 MHz`.
- AHB clock runs at `200 MHz`.
- APB1/APB2/APB3/APB4 run at `100 MHz`.
- HAL time base uses `TIM7`, not SysTick.
- The application enables the Cortex-M7 DWT cycle counter for microsecond delays in the CS5552 driver.
- The generated scatter file defines:
  - `IROM1`: `0x08000000`, size `0x00020000`
  - `IRAM1`: `0x20000000`, size `0x00020000`
  - `IRAM2`: `0x24000000`, size `0x00080000`
- Current map report shows about `28.71 kB` RO and `10.05 kB` RW+ZI usage.

## Runtime Model

- Startup order in `main()` is:
  - `MPU_Config()`
  - `HAL_Init()`
  - `SystemClock_Config()`
  - `MX_*_Init()` for GPIO, DMA, timers, I2C, SPI, USART
  - DWT cycle counter enable
  - `CS5552_Init()` for chip 0 and chip 1
  - start continuous conversion on both chips
  - initialize two sliding-average filters
  - enter an infinite polling loop
- There is no RTOS in the current project.
- `printf()` is redirected through `fputc()` to blocking `HAL_UART_Transmit(&huart1, ...)`.
- Although DMA is configured for `USART1`, current application output is blocking UART, not DMA-driven.

## Active Application Logic

- Main application code lives in:
  - `Core/Src/main.c`
  - `Core/Src/user.c`
  - `Core/Src/filter.c`
- `user.c` implements the CS5552 driver:
  - multi-chip CS selection between `P_ADCCS2` and `P_ADCCS3`
  - SPI command parity generation
  - register read/write
  - software reset sequence
  - offset self-calibration
  - continuous/single conversion helpers
  - raw-code to voltage conversion
- The driver uses `SPI3` through `hspi3`.
- The CS5552 data-ready polling is done by reading `PC11`, which is also the `SPI3_MISO` pin.
- Filtering uses a 10-sample sliding average (`FILTER_SIZE = 10`).

## Current ADC Behavior

- The firmware initializes two CS5552 chips and configures both channels internally.
- `CS5552_Init()` writes zero offset, unity gain calibration, conversion config, system config, then runs offset self-calibration for both logical channels.
- The configured conversion settings in `user.c` are currently `3200 Hz` and `64x` gain for both channels.
- In the main loop, only chip 0 data is actively processed and printed.
- Chip 1 read/print logic is present but commented out.
- The main loop reads `REG_CONV_DATA`, rejects several obvious invalid raw patterns, converts the value, applies the sliding-average filter, and prints the filtered result.
- The sample-rate counter prints once per second as `rate is ...`.
- The polling condition `HAL_GetTick() - adc_tick >= 0` is always true for unsigned tick math, so the loop currently polls as fast as it can rather than at a throttled interval.

## Peripheral Inventory

- Clearly active in current app behavior:
  - `GPIO`
  - `SPI3` for CS5552
  - `USART1` for console/data output
  - `TIM7` as HAL tick source
  - DWT cycle counter
- Configured in CubeMX/Keil but not meaningfully used by current application logic:
  - `SPI1` for W5500 signals
  - `SPI2` for DAC signals
  - `I2C2`
  - `USART2`
  - `TIM1`, `TIM4`, `TIM5` encoder interfaces
  - `TIM2`, `TIM8`, `TIM15` PWM-capable timers
  - `TIM6`, `TIM13`, `TIM14`, `TIM16`, `TIM17` base timers
  - `USART1` DMA RX/TX streams
- `P_ADCDRDY` on `PD2` is configured as falling-edge EXTI, but current application logic does not use that interrupt path for ADC servicing.

## Board and Signal Notes

- `main.h` contains the CubeMX-generated board pin naming.
- Important CS5552-related pins:
  - `P_ADCCS2` on `PD3`
  - `P_ADCCS3` on `PD0`
  - `P_ADCDRDY` on `PD2`
  - `SPI3`: `PC10/PC11/PC12`
- Other named hardware present in the pin map suggests a larger board design with:
  - W5500 Ethernet signals
  - MRAM control pins
  - DAC SPI signals
  - multiple digital inputs/outputs
  - encoder-related A/B signals
- Many of these board resources are only configured at the GPIO/peripheral-init level so far and are not yet connected to application logic.

## Configuration Drift To Remember

- `shiyanji_H7.ioc` says `SPI3.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16`.
- `Core/Src/spi.c` currently sets `hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8`.
- When regenerating from CubeMX, verify whether this was an intentional manual edit before accepting generated changes.

## VSCode Caveat

- `.vscode/c_cpp_properties.json` is not aligned with the real embedded build:
  - it points IntelliSense to `cl.exe`
  - it uses placeholder include paths like `STM32xxxx`
  - it is useful only as a loose editor hint
- Keil project files remain the source of truth for actual compilation.

## Safe Editing Guidance For Future Agents

- Prefer edits in `Core/Src` and `Core/Inc`.
- Keep new application logic inside CubeMX `USER CODE BEGIN/END` regions when modifying generated files.
- Avoid editing `Drivers/` unless the task specifically requires HAL/CMSIS changes.
- Before using `.ioc` regeneration, compare generated output with current hand edits in `main.c`, `spi.c`, `user.c`, and related init files.
- If changing ADC behavior, inspect both `main.c` and `user.c`; the main loop currently reflects only one of the two chips even though both are initialized.
