# 位置通道标定说明

## 1. 文档目的

本文整理 `poseCalcu()`、`SenData[ch0Pose].LinPoint`、`LinV[]`、`multipointCalibrateFunc()` 与 `singlepointCalibrateFunc()` 的关系，用于说明位置通道 `pose.code -> pose.orig` 的标定换算流程。

相关源码位置：

- `Host/in_out.c`
- `Host/sensor.c`
- `Host/sensor.h`

## 2. 上层调用关系

`poseCalcu()` 的核心逻辑是：

```c
void poseCalcu(void){
    pose.orig_last = pose.orig;
    if(SenData[ch0Pose].LinPoint > 0){
        sensorCalibrate.multipointCalibrate(pose.code, ch0Pose, &SenData[ch0Pose], &pose.orig);
    }
    else{
        sensorCalibrate.singlepointCalibrate(pose.code, ch0Pose, &SenData[ch0Pose], &pose.orig);
    }
}
```

可见，位置工程量 `pose.orig` 的来源分两种：

- `LinPoint > 0`：走多点标定 `multipointCalibrate`
- `LinPoint == 0`：走单点标定 `singlepointCalibrate`

也就是说，`LinPoint` 是“是否启用线性化标定表”的开关。

## 3. `SenData[ch0Pose].LinPoint` 的作用

`LinPoint` 表示当前传感器已经配置了多少个线性化标定点。

它的判定意义如下：

| LinPoint 值 | 含义 | 换算方式 |
| --- | --- | --- |
| `0` | 未配置多点线性化表 | 使用单点灵敏度公式 |
| `> 0` | 已配置线性化表 `LinV[]` | 使用分段线性标定 |

`LinPoint` 不是实时算法计算值，而是配置参数。它通常来自：

- 默认初始化
- MRAM 恢复
- EEPROM 读取
- 上位机下发新的传感器标定数据

## 4. `LinV[]` 标定表结构

`LinV[]` 可以理解成一张“分段线性标定表”。表中每一行描述一个参考标定点及其对应换算参数。

### 4.1 三列对应关系图

```text
LinV[] 标定表

+---------+-------------------+-------------------+------------------------+
| 索引 i  | ADCCode           | RefValue          | CorrectionFactor       |
+---------+-------------------+-------------------+------------------------+
| 0       | LinV[0].ADCCode   | LinV[0].RefValue  | LinV[0].CorrectionFactor |
| 1       | LinV[1].ADCCode   | LinV[1].RefValue  | LinV[1].CorrectionFactor |
| 2       | LinV[2].ADCCode   | LinV[2].RefValue  | LinV[2].CorrectionFactor |
| ...     | ...               | ...               | ...                    |
| n-1     | LinV[n-1].ADCCode | LinV[n-1].RefValue| LinV[n-1].CorrectionFactor |
+---------+-------------------+-------------------+------------------------+
```

### 4.2 每一列的含义

#### `ADCCode`

- 含义：该标定点对应的原始码值
- 用途：用于判断当前 `_code` 落在哪个标定分段

#### `RefValue`

- 含义：该标定点对应的参考工程量
- 用途：作为分段线性换算中的基准工程值

#### `CorrectionFactor`

- 含义：该段的修正系数或斜率
- 用途：把“码值差”换算成“工程量差”

## 5. 多点标定 `multipointCalibrateFunc()`

### 5.1 作用

`multipointCalibrateFunc()` 用于在已经配置 `LinV[]` 标定表时，将采样码值 `_code` 按“分段线性插值”的方式转换为工程值。

它适合以下场景：

- 传感器或前端链路不完全线性
- 需要分段校正不同量程下的灵敏度误差
- 单一固定比例无法满足精度要求

### 5.2 核心公式

函数核心公式为：

```text
orig = (code - m) * k + n
```

其中：

- `code`：当前输入码值 `_code`
- `m`：当前参考段的 `ADCCode`
- `n`：当前参考段的 `RefValue`
- `k`：当前参考段的 `CorrectionFactor`

### 5.3 处理流程

多点标定大致分为三步：

1. 根据 `_code` 在 `LinV[]` 中找到所属分段
2. 取出该分段的 `ADCCode / RefValue / CorrectionFactor`
3. 使用分段线性公式换算出工程值

### 5.4 查表逻辑

函数会先检查：

```c
_senData->LinV[0].CorrectionFactor < 0.0f
```

这一步的目的，是判断标定表整体是“正斜率”还是“负斜率”，从而决定后续如何比较 `_code` 和 `ADCCode`。

#### 正斜率时

- 逐个查找第一个满足 `_code <= ADCCode` 的点
- 如果没有找到，就使用最后一个点

#### 负斜率时

- 逐个查找第一个满足 `_code > ADCCode` 的点
- 如果没有找到，就使用第一个点

这说明：函数并不是简单地“固定取第 0 个点”，而是根据当前码值动态选择对应分段。

### 5.5 通道后处理

换算得到 `_orig` 后，还会根据不同通道做额外处理：

#### `ch0Pose`

- 只做方向修正
- 公式：

```text
orig = AL.posCtrl.sign * orig
```

#### `ch2Ext1 / ch3Ext2 / ch4Load`

- 先保存未去皮的工程值到 `origTare`
- 再减去浮点 tare 值 `AL.tare.fValue[]`

### 5.6 对位置通道的含义

对 `pose` 而言，多点标定可以理解为：

```text
pose.code
  -> 在 LinV[] 中查找所在分段
  -> 取该分段 ADCCode / RefValue / CorrectionFactor
  -> 分段线性换算
  -> 乘以位置方向 AL.posCtrl.sign
  -> 得到 pose.orig
```

## 6. 单点标定 `singlepointCalibrateFunc()`

### 6.1 作用

`singlepointCalibrateFunc()` 用于在没有配置线性化表时，按额定灵敏度直接将原始码值换算成工程量。

它适合以下场景：

- 没有配置 `LinV[]`
- 只需要简单的线性比例换算
- 先实现基本功能，再逐步升级为多点标定

### 6.2 对位置通道的核心公式

对 `ch0Pose`，函数逻辑本质上是：

```text
orig = AL.posCtrl.sign * code * AL.posCtrl.NominalSensitive
```

含义是：

- `code`：当前累计位置码值
- `NominalSensitive`：单位码值对应的工程量灵敏度
- `sign`：方向符号，用于统一正负方向

### 6.3 其他通道

对 `ch2Ext1 / ch3Ext2 / ch4Load`，函数会结合：

- 前端 ADC 系数
- 额定灵敏度
- 额定量程
- tare 去皮

得到最终工程量。

### 6.4 对位置通道的含义

对 `pose` 而言，单点标定可以理解为：

```text
pose.code
  -> 直接乘位置灵敏度 NominalSensitive
  -> 再乘方向符号 AL.posCtrl.sign
  -> 得到 pose.orig
```

它不依赖 `LinV[]`，也不做分段插值。

## 7. 多点标定与单点标定对比

| 项目 | 多点标定 `multipointCalibrate` | 单点标定 `singlepointCalibrate` |
| --- | --- | --- |
| 使用条件 | `LinPoint > 0` | `LinPoint == 0` |
| 数据来源 | `LinV[]` 标定表 | 单一灵敏度参数 |
| 数学方式 | 分段线性换算 | 固定比例换算 |
| 精度 | 更高，适合补偿非线性 | 较基础，依赖单点灵敏度 |
| 计算复杂度 | 略高 | 更简单 |
| 典型用途 | 精细标定 | 默认换算、基础运行 |

可以简单记成：

- `singlepointCalibrate`：整段量程只用一个比例
- `multipointCalibrate`：整段量程拆成若干小段，每段有自己的比例

## 8. 位置通道完整数据流

对 `pose` 这一条链，位置工程量的来源可以整理成：

```text
Encoder 计数
  -> pose.code
  -> poseCalcu()
       -> 判断 SenData[ch0Pose].LinPoint
       -> LinPoint > 0 : multipointCalibrate()
       -> LinPoint = 0 : singlepointCalibrate()
  -> pose.orig
  -> controlLoop()
```

如果进一步展开：

```text
编码器脉冲
  -> TIM 编码器计数
  -> Encoder_Get()
  -> pose.code
  -> poseCalcu()
  -> 标定换算
  -> pose.orig
  -> 位置闭环控制
```

## 9. 总结归类

### 9.1 配置类

- `SenData[ch0Pose].LinPoint`
- `SenData[ch0Pose].LinV[]`
- `AL.posCtrl.NominalSensitive`
- `AL.posCtrl.sign`

这一类决定“标定参数是什么”。

### 9.2 换算类

- `multipointCalibrateFunc()`
- `singlepointCalibrateFunc()`
- `poseCalcu()`

这一类决定“如何把码值换成工程量”。

### 9.3 数据流类

- `pose.code`
- `pose.orig`
- `pose.orig_last`

这一类表示“换算前后的过程量”。

### 9.4 控制使用类

- `pose.orig`
- `speedPose.filter`
- `controlLoop()`

这一类表示“换算结果最终如何进入控制环”。

## 10. 结论

`poseCalcu()` 的核心不是简单乘一个比例，而是先判断当前位置通道是否配置了线性化标定表：

- 配置了 `LinV[]`，就用 `multipointCalibrate()` 做分段线性换算
- 没配置 `LinV[]`，就用 `singlepointCalibrate()` 做固定比例换算

因此，`LinPoint` 和 `LinV[]` 决定了位置通道标定精度的上限，而 `singlepointCalibrate()` 则是没有多点标定时的基础退化路径。
