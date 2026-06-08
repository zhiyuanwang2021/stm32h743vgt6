# CS5552 Data Flow

## 1. `force.filter` 数据流

```text
CS5552 chip0.ch1
  |
  |  continuous conversion data ready
  v
CS5552_ReadReg(REG_CONV_DATA, &adc_raw)
  |
  |  in Hardware/CS5552.c
  v
CS5552_ReadChipContDataNonBlocking(...)
  |
  |-- parse raw register value
  |-- extract adc_24bit
  |-- CS5552_ConvertToVoltage(...)
  |-- Filter_Update(&voltage_filter_ch0, voltage_mv)
  v
cs5552_compat_ctx.Code[CS5552_COMPAT_CHANNEL_FORCE]
cs5552_compat_ctx.Voltage[CS5552_COMPAT_CHANNEL_FORCE]
cs5552_compat_ctx.Value[CS5552_COMPAT_CHANNEL_FORCE]
  |
  |  called by Host/in_out.c -> inputGetValue()
  v
CS5552_CompatDataGet(&cs5552_compat_ctx)
  |
  |  called by Host/in_out.c -> inputMapping()
  v
MD_CODE_map(sensorCheck.MapNum)
  |
  |-- if no load sensor: force.code = 0
  |-- else:
  v
force.code = cs5552_compat_ctx.Code[CS5552_COMPAT_CHANNEL_FORCE]
             - AL.tare.value[ch4Load]
  |
  v
loadCalcu(&force, &AL)
  |
  |-- sensor scaling
  |-- nominal value / sensitivity conversion
  |-- tare and sign handling
  v
force.orig
  |
  v
loadFilterLsm(...)
loadFilterWithLenSwitch(...)
  |
  v
force.filter
```

## 2. `strain2.filter` 数据流

```text
CS5552 chip1.ch0
  |
  |  continuous conversion data ready
  v
CS5552_ReadReg(REG_CONV_DATA, &adc_raw)
  |
  v
CS5552_ReadChipContDataNonBlocking(...)
  |
  |-- parse adc_raw
  |-- extract adc_24bit
  |-- CS5552_ConvertToVoltage(...)
  |-- Filter_Update(&voltage_filter_ch1, voltage_mv)
  v
cs5552_compat_ctx.Code[CS5552_COMPAT_CHANNEL_STRAIN2]
cs5552_compat_ctx.Voltage[CS5552_COMPAT_CHANNEL_STRAIN2]
cs5552_compat_ctx.Value[CS5552_COMPAT_CHANNEL_STRAIN2]
  |
  v
CS5552_CompatDataGet(&cs5552_compat_ctx)
  |
  v
MD_CODE_map(sensorCheck.MapNum)
  |
  |-- if no strain2 sensor: strain2.code = 0
  |-- else:
  v
strain2.code = cs5552_compat_ctx.Code[CS5552_COMPAT_CHANNEL_STRAIN2]
               - AL.tare.value[ch3Ext2]
  |
  v
extensometer2Calcu(&strain2, &AL)
  |
  |-- calibration
  |-- nominal sensitive / nominal value conversion
  |-- sign / tare handling
  v
strain2.orig
  |
  v
extensometer2FilterLsm(...)
extensometer2FilterWithLenSwitch(...)
  |
  v
strain2.filter
```

## 3. 主循环总链路

```text
LoopTask (about 1 ms)
  |
  v
inputGetValue()
  |
  |-- CS5552_CompatDataGet(&cs5552_compat_ctx)
  |-- DIDetect_ReadALL()
  |-- DB9Detect_ReadALL()
  v
raw acquisition buffers updated
  |
  v
inputMapping()
  |
  |-- DI_map / DIFunc_map / ServoIO_map / DB9_map
  |-- sensorCON_ADJ_Monitor(...)
  |-- MD_CODE_map(...)
  |-- SetTareMonitor(&AL)
  |-- poseCalcu()
  |-- bigDeformationCalcu(...)
  |-- posespeed_filter(...)
  |-- loadCalcu(&force, &AL)
  |-- loadFilterLsm / loadFilterWithLenSwitch
  |-- extensometerCalcu(&strain1, &AL)
  |-- extensometerFilter...
  |-- extensometer2Calcu(&strain2, &AL)
  |-- extensometer2Filter...
  |-- loadspeed_filter(...)
  |-- sendata_pg_process()
  v
application-layer measurements ready
  |
  |  examples:
  |-- pose.orig / pose.filter
  |-- force.orig / force.filter
  |-- strain1.orig / strain1.filter
  |-- strain2.orig / strain2.filter
  v
controlLoop()
  |
  |-- choose mode:
  |   POS / LOAD / EXTEN
  |-- run PID / ADRC / trajectory logic
  |-- compute output command
  v
outputMapping()
  |
  |-- openloopOutputMap(...) or closeloopOutputMap(...)
  |-- SONMap()
  v
outputControl()
  |
  |-- Servo_mod_fclk(...)
  |-- DAC8831_Write(...)
  |-- DIR / SON output
  |-- DO / Relay output
```

## 4. `DEBUG_LOAD_SENSOR` 字段对照表

| 字段名 | 来源函数/位置 | 物理含义 |
|---|---|---|
| `force.code` | `Host/in_out.c` `MD_CODE_map()` | 载荷通道的原始 ADC 码值，已减去整数去皮值 |
| `force.orig` | `Host/in_out.c` `loadCalcu()` | 由 `force.code` 经过标定参数换算后的原始载荷工程量 |
| `force.filter` | `Host/in_out.c` `loadFilterWithLenSwitch()` | 当前主载荷滤波结果，控制和显示常用值 |
| `forceLsm.filter` | `Host/in_out.c` `loadFilterLsm()` | 最小二乘/平滑链路输出的载荷滤波值 |
| `force.filterTrans` | `Host/in_out.c` 载荷映射/传输流程 | 传输或显示侧使用的一路载荷值 |
| `AL.tare.value[ch4Load]` | `SetTareMonitor()` / 全局参数 `AL.tare` | 载荷通道整数去皮值，直接作用于 `force.code` |
| `AL.tare.fValue[ch4Load]` | `SetTareMonitor()` / 全局参数 `AL.tare` | 载荷通道浮点去皮值，作用于工程量换算链 |
| `AL.loadCtrl.NominalSensitive` | 全局参数 `AL.loadCtrl` | 载荷传感器额定灵敏度，标定换算参数 |
| `AL.loadCtrl.NominalValue` | 全局参数 `AL.loadCtrl` | 载荷传感器额定量程/额定值，标定换算参数 |
| `filterLen.loadLen` | `Host/in_out.c` `loadFilterSwitch()` | 当前实际使用的载荷滤波长度 |
| `SenData[ch4Load].sensorIntgr` | 传感器参数 `SenData[]` | 载荷传感器配置里的积分/滤波相关参数 |

