# EasyLogger Notes For This Project

基于当前工程代码通读整理，时间：2026-06-04。

## 1. 当前工程里 EasyLogger 是怎么接上的

这个工程里的 EasyLogger 不是“仅拷贝源码未启用”，而是已经完成了 FreeRTOS 适配，并且实际在业务代码中使用。

主链路如下：

1. 业务模块通过 `log_e/log_w/log_i/log_d/log_v` 宏打日志。
2. `Core/Src/freertos.c` 在 `MX_FREERTOS_Init()` 一开始调用 `Static_Set_Output_Log_Format()`。
3. `Static_Set_Output_Log_Format()` 位于 `Easylogger/port/elog_port.c`，内部依次执行：
   - `elog_init()`
   - `elog_set_fmt(...)`
   - `elog_start()`
4. `elog_init()` 会继续完成：
   - `elog_port_init()`：创建日志互斥量
   - `elog_async_init()`：创建异步输出信号量和异步日志任务
5. 最终输出函数是 `elog_port_output()`，它内部直接调用 `printf("%.*s", size, log)`。
6. 当前工程 `printf` 已在 `Core/Src/usart.c` 重定向到 `fputc()`，并通过 `HAL_UART_Transmit(&huart1, ...)` 发到 `USART1`。

结论：当前 EasyLogger 的最终物理输出通道是 `USART1`，波特率是 `921600`。

## 2. 当前配置

`Easylogger/inc/elog_cfg.h` 里的关键配置如下：

- 已开启日志输出：`ELOG_OUTPUT_ENABLE`
- 编译期最高日志级别：`ELOG_OUTPUT_LVL = ELOG_LVL_WARN`
- 已开启断言：`ELOG_ASSERT_ENABLE`
- 单行日志缓冲区：`ELOG_LINE_BUF_SIZE = 1024`
- Tag 最大长度：`20`
- 关键字过滤最大长度：`16`
- Tag 级别过滤表最大数量：`5`
- 换行符：`"\r\n"`
- 已开启彩色输出：`ELOG_COLOR_ENABLE`
- 已开启异步输出：`ELOG_ASYNC_OUTPUT_ENABLE`
- 异步模式使用 FreeRTOS：`ELOG_ASYNC_OUTPUT_USING_FREERTOS`
- 异步缓冲区大小：`ELOG_ASYNC_OUTPUT_BUF_SIZE = ELOG_LINE_BUF_SIZE * 10`
- 已开启“按行取日志”：`ELOG_ASYNC_LINE_OUTPUT`
- 缓冲输出模式未启用：`ELOG_BUF_OUTPUT_ENABLE` 被注释掉

## 3. 这套实现的运行机制

### 3.1 同步保护

`Easylogger/port/elog_port.c` 在 `elog_port_init()` 中创建了一个 `osMutex`，句柄名为 `elogHandle`。  
`elog_port_output_lock()` / `elog_port_output_unlock()` 用它来保护多任务并发打印，避免多条日志交叉。

### 3.2 异步输出

`Easylogger/src/elog_async_freertos.c` 里做了 FreeRTOS 版本异步输出：

- 用一个环形缓冲区缓存日志
- 用一个信号量 `output_noticeHandle` 通知后台输出任务
- 创建一个后台任务 `elog_async_task`
- 后台任务被唤醒后循环取日志，再调用 `elog_port_output()`

这意味着：

- 业务任务大多不直接卡在 UART 发串口上
- 真正的串口输出仍然是阻塞式 `printf -> HAL_UART_Transmit`
- 所以“异步”主要是把阻塞从业务任务挪到了日志任务，不是 DMA 式非阻塞输出

### 3.3 时间和线程信息

`elog_port_get_time()` 用的是 FreeRTOS `xTaskGetTickCount()`。  
`elog_port_get_t_info()` 返回当前任务名 `pcTaskGetName(xTaskGetCurrentTaskHandle())`。  
`elog_port_get_p_info()` 当前返回空字符串。

因此，这个工程的日志支持：

- 时间戳
- 当前任务名
- 不提供进程信息

## 4. 当前日志格式策略

`Static_Set_Output_Log_Format()` 对不同级别做了静态格式配置：

- `ASSERT`：`ELOG_FMT_ALL`
- `ERROR`：级别 + 时间 + 线程信息
- `WARN`：级别 + 时间 + 线程信息
- `INFO`：级别 + 时间 + 线程信息
- `DEBUG`：几乎全部信息，但去掉函数名和进程信息
- `VERBOSE`：`ELOG_FMT_ALL`

这说明当前项目更偏向：

- `I/W/E` 用于运行期简洁状态日志
- `D/V` 用于更完整的定位信息

## 5. 一个容易忽略但很重要的点

`elog_cfg.h` 里定义的是：

- `ELOG_ASYNC_OUTPUT_LVL = ELOG_LVL_ASSERT`

而 `elog_async_freertos.c` 里的实现写成了特殊分支：

- 如果异步级别等于 `ASSERT`，则所有级别都进入异步输出路径

也就是说，在这个工程里：

- 不是“只有 assert 异步”
- 而是“所有 EasyLogger 日志都走异步缓冲 + 后台任务输出”

这是当前项目代码行为里的一个关键细节。

## 6. 当前工程里是怎么用 EasyLogger 的

工程中有大量业务代码直接使用 `log_e/log_i/log_d`，主要分布在：

- `Host/control.c`
- `Host/communicate.c`
- `Host/gParameter.c`
- `Host/in_out.c`
- `Host/mram_manage.c`
- `Host/posGenerator.c`
- `Host/sensor.c`
- `Host/Eeprom_manage.c`
- `Host/EthProtocol.c`
- `Core/Src/freertos.c`
- `Ethenet/ETHw5500.c`
- `Hardware/RS485.c`
- `Hardware/DI.c`
- `Hardware/Servo_driver.c`
- `Hardware/myFifo.c`

但同时也存在很多仍然直接用 `printf()` 的地方，例如：

- `Host/communicate.c`
- `Core/Src/freertos.c`
- `Hardware/CS5552.c`
- `Ethenet/ETHw5500.c`
- `Host/Eeprom_manage.c`
- `Host/mram_manage.c`

所以当前项目的日志现状不是“全部统一成 EasyLogger”，而是：

- 一部分走 `EasyLogger`
- 一部分仍走原始 `printf`

## 7. LOG_TAG / LOG_LVL 的实际情况

`Easylogger/inc/elog.h` 里给了默认值：

- `LOG_TAG = "FreeRTOS_TAG"`
- `LOG_LVL = ELOG_LVL_VERBOSE`

我在当前工程业务代码里没有找到自定义的 `#define LOG_TAG` 或 `#define LOG_LVL`。  
这意味着当前大多数 `log_x(...)` 宏如果没有单独定义 tag，就会使用默认 tag。

结论：

- 当前工程里的大多数 EasyLogger 日志标签大概率是统一默认 tag
- 项目尚未把 tag 细分到各模块

如果以后想让日志更好筛选，最直接的改法是在各模块 `#include <elog.h>` 之前定义自己的 `LOG_TAG`。

## 8. `elog_init_ok` 在项目中的实际用途

`elog.c` 在 `elog_start()` 里把全局标志 `elog_init_ok` 置为 `true`。  
当前工程中，`Host/mram_manage.c` 会在部分路径里先判断 `elog_init_ok`，再决定是否使用 `log_e/log_d`。

这说明项目已经意识到：

- 某些初始化早期路径可能还不能安全依赖 EasyLogger
- 因而保留了部分 `printf` 兜底

## 9. 随库带来的高级能力

EasyLogger 目录里除了核心日志外，还带了这些能力：

- `elog_set_filter*`：按级别 / tag / keyword 过滤
- `elog_hexdump()`：十六进制输出
- `plugins/flash/`：Flash 日志插件
- `plugins/file/`：文件日志插件
- `elog_buf.c`：缓冲输出模式

但就当前固件工程来看：

- 这些高级接口大多“存在于库中”
- 业务代码里没有看到实际启用或调用

尤其是：

- `flash` 插件未接入当前业务
- `file` 插件未接入当前业务
- 过滤接口存在，但项目里没有看到主动设置过滤规则
- `hexdump` 接口存在，但项目里没看到调用点

## 10. 这个实现的优点

- 已支持 FreeRTOS 多任务环境
- 已用互斥量避免日志交叉
- 已有异步输出任务，能减少业务任务直接阻塞
- 已支持任务名输出，定位任务上下文更方便
- 接口使用成本低，业务代码只要 `log_i/log_e/...` 即可

## 11. 这个实现的限制和注意点

### 11.1 输出后端仍然偏重

当前 `elog_port_output()` 还是走 `printf`，而 `printf` 最终又是：

- `fputc()`
- `HAL_UART_Transmit()`
- 单字符阻塞发送

所以日志量大时，后台异步任务本身仍可能消耗明显 CPU 时间。

### 11.2 浮点日志会带来较大代码体积

项目里很多 `log_i(... "%f" ...)` 和 `printf("%f" ...)`。  
在 ARMCC/嵌入式场景下，浮点格式化通常会明显增大代码体积，这一点已经和当前 Flash 超限问题直接相关。

### 11.3 模块 tag 尚未细分

当前看不到模块级 `LOG_TAG` 自定义，后续如果需要现场排障、按模块过滤日志，现状不够友好。

### 11.4 `elog_port_get_time()` 显示的是 RTOS tick 视角

它不是 RTC 时间，也不是绝对日期时间。  
它更适合看“系统运行多久后发生了什么”。

## 12. 对当前工程的理解总结

当前工程里的 EasyLogger 可以理解为：

- 已接好
- 已在业务中广泛使用
- 已做 FreeRTOS 异步输出适配
- 最终仍依赖 `printf + USART1` 输出
- 仍与大量历史 `printf` 并存

如果后续要继续演进，通常会优先考虑这几个方向：

1. 给各业务模块补 `LOG_TAG`
2. 把部分高频 `printf` 迁移到 `log_x`
3. 对发布版减少浮点日志和超长字符串
4. 如果日志很多，考虑把串口输出从阻塞发送继续优化为 DMA 或更轻量的输出通道

## 13. EasyLogger 精简优化建议

这一节专门针对当前的 Flash 超限问题。  
你这次遇到的是 `LR_IROM1` 超出上限，按构建报错看，当前只需要回收很小一部分 Flash 空间就能重新通过链接。

先说结论：

- 对这次“只超一点点”的问题，最划算的通常不是大改架构
- 而是优先裁掉调试日志、浮点格式化、参数回显字符串
- EasyLogger 本体也能裁，但真正的大头往往还是业务文件里的 `printf("%f ...")` 和长日志文本

## 14. 先看证据：EasyLogger 自身大概占了什么

从当前 `shiyanji_H7.map` 可以看出：

- `elog.o(i.elog_output)` 代码约 `0x37c`，约 `892` 字节
- `elog_async_freertos.o` 的异步相关代码整体也就几百字节级
- `elog_port_get_time()` 只有 `0x44`，约 `68` 字节
- `elog_port_output()` 只有 `0x10`，约 `16` 字节

这说明：

- EasyLogger 核心本体并不算特别大
- 但它会把一整套“格式化、异步、颜色、任务名、时间戳”逻辑都带进来
- 更重要的是，业务层的大量日志字符串和浮点格式化，才是更容易把 Flash 顶满的部分

同时从 map 还能看到：

- `elog.o(.bss)` 约 `1212` 字节
- `elog_async_freertos.o(.bss)` 约 `11260` 字节

这两项主要是 RAM 压力，不是这次 Flash 超限的直接根因。  
所以像 `ELOG_LINE_BUF_SIZE` 这类参数，优先影响的是 RAM，不是当前这个链接错误。

## 15. 按收益排序的裁剪建议

### 15.1 第一优先级：裁掉业务侧高频调试打印

这是最推荐的第一刀，因为：

- 改动最小
- 风险最低
- 最容易立刻回收几百字节到几 KB

当前最值得优先处理的文件：

- `Host/communicate.c`
- `Core/Src/freertos.c`
- `Hardware/CS5552.c`
- `Ethenet/ETHw5500.c`
- `Host/mram_manage.c`
- `Host/Eeprom_manage.c`

尤其是下面这些类型最值得先裁：

- 参数回显类 `printf`
- 批量状态打印
- 初始化成功类提示
- 高频运行过程打印
- 带 `%f` 的调试输出

例如：

- `Host/communicate.c` 中大量“命令收到后把整组参数逐项打印出来”的代码
- `Core/Src/freertos.c` 中任务列表、运行统计、控制环观测量打印
- `Hardware/CS5552.c` 中初始化和异常过程的大量诊断文本
- `Ethenet/ETHw5500.c` 中网络参数、socket 状态、FIFO dump 打印

如果只是为了先让链接通过，直接把这些打印改成条件编译，收益通常最大。

建议做法：

```c
#if ENABLE_DEBUG_PRINT
printf("...");
#endif
```

或者：

```c
#if ENABLE_DEBUG_LOG
log_i("...");
#endif
```

### 15.2 第二优先级：减少或禁止 `%f` / `%lf`

这个点对 Flash 很敏感。

当前工程里：

- `Host/communicate.c` 有大量 `%f`
- `Core/Src/freertos.c` 有大量 `%f` / `%.10f` / `%.15f`
- 一部分 `log_i()` 里也带浮点参数

在 ARMCC + `printf` 场景下，浮点格式化通常非常贵。  
如果当前超限只有几百字节，删掉几处浮点打印，往往就足够了。

建议：

- 发布版尽量不要打印浮点
- 如必须打印，可改成放大后的整数
- 或者只保留最关键的 1 到 2 个错误场景

例如把：

```c
printf("speed=%f\r\n", speed);
```

改成：

```c
printf("speed_x1000=%ld\r\n", (long)(speed * 1000));
```

或者发布版直接不打印。

### 15.3 第三优先级：降低 EasyLogger 编译级别

当前配置是：

```c
#define ELOG_OUTPUT_LVL ELOG_LVL_VERBOSE
```

这意味着：

- `log_a`
- `log_e`
- `log_w`
- `log_i`
- `log_d`
- `log_v`

全部都在编译期保留下来。

如果改成：

```c
#define ELOG_OUTPUT_LVL ELOG_LVL_INFO
```

则：

- `log_d`
- `log_v`

在预处理阶段就会被裁掉。

如果改成：

```c
#define ELOG_OUTPUT_LVL ELOG_LVL_WARN
```

则：

- `log_i`
- `log_d`
- `log_v`

都会被直接裁掉。

这类裁剪对 Flash 很有效，因为它不只是少执行日志，而是连很多调用点和字符串都可能一起消失。

推荐策略：

- 调试版：`ELOG_LVL_VERBOSE` 或 `ELOG_LVL_DEBUG`
- 发布版：`ELOG_LVL_WARN`，最多 `ELOG_LVL_INFO`

### 15.4 第四优先级：关闭 EasyLogger 异步输出

当前启用了：

```c
#define ELOG_ASYNC_OUTPUT_ENABLE
#define ELOG_ASYNC_OUTPUT_USING_FREERTOS
```

这会带入：

- 环形缓冲区管理
- 通知信号量
- 后台日志任务
- 一整套异步取日志逻辑

如果当前目标是：

- 先解决 Flash 超限
- 发布版日志量不大
- 不强依赖多任务异步输出

那么可以考虑在发布版关闭异步输出。

建议改成：

```c
//#define ELOG_ASYNC_OUTPUT_ENABLE
```

这样会直接裁掉：

- `elog_async_freertos.c` 相关代码
- 一部分初始化和异步输出路径

代价是：

- 日志重新变成同步输出
- 高频日志时更容易阻塞任务

所以这个建议更适合：

- 发布版
- 低日志量版本
- 临时救急通过链接

### 15.5 第五优先级：关闭彩色输出

当前启用了：

```c
#define ELOG_COLOR_ENABLE
```

如果串口终端并不真正依赖 ANSI 颜色，发布版可以考虑关闭：

```c
//#define ELOG_COLOR_ENABLE
```

这类收益通常不如裁日志调用点大，但属于：

- 改动小
- 风险小
- 顺手可做

它主要能去掉：

- 颜色控制字符串
- 部分彩色输出逻辑分支

### 15.6 第六优先级：保留 EasyLogger，但简化输出策略

当前项目对 `ERROR/WARN/INFO` 都配置了：

- 级别
- 时间
- 线程信息

这本身对 Flash 的帮助不会像裁日志那样明显，但可以作为收尾优化考虑。

更重要的一点是：

- 即使你把格式项配少了
- `elog_output()` 这个大函数仍然还会被保留

所以“只改格式集合”通常不是第一优先级的 Flash 优化手段。  
它更偏向运行期输出简化，不是最强的代码体积裁剪手段。

## 16. 哪些配置改了主要省 RAM，不要误判成省 Flash

下面这些项值得调，但主要作用不是当前这个链接错误：

- `ELOG_LINE_BUF_SIZE`
- `ELOG_ASYNC_OUTPUT_BUF_SIZE`
- `ELOG_ASYNC_POLL_GET_LOG_BUF_SIZE`

例如当前：

- `elog.o` 里有 `1024` 字节日志缓冲
- `elog_async_freertos.o` 里有 `10240` 字节环形缓冲
- 还有约 `1020` 字节的轮询取日志缓冲

这些压的是 RAM。  
如果后面要优化内存占用，很值得改；但针对这次 Flash 超限，不应把它当成主手段。

## 17. 结合当前工程，推荐的最小代价方案

如果目标只是“先把当前链接错误解决掉”，我推荐的顺序是：

1. 先把 `Core/Src/freertos.c` 里的任务列表/运行统计/观测量打印关掉
2. 再裁 `Host/communicate.c` 里大段参数回显 `printf`
3. 再裁 `Hardware/CS5552.c` 和 `Ethenet/ETHw5500.c` 的初始化成功类和调试类打印
4. 把 `ELOG_OUTPUT_LVL` 从 `VERBOSE` 降到 `INFO` 或 `WARN`
5. 如果还差一点，再关闭 `ELOG_COLOR_ENABLE`
6. 仍不够时，再考虑关闭 `ELOG_ASYNC_OUTPUT_ENABLE`

这个顺序的好处是：

- 先动业务调试输出，最容易见效
- 再动 EasyLogger 配置，回收额外空间
- 尽量不一上来就牺牲日志体系结构

## 18. 面向发布版的建议配置

如果希望做一个“发布版日志精简配置”，我建议优先尝试：

```c
#define ELOG_OUTPUT_ENABLE
#define ELOG_OUTPUT_LVL ELOG_LVL_WARN
#define ELOG_ASSERT_ENABLE
#define ELOG_LINE_BUF_SIZE 1024
//#define ELOG_COLOR_ENABLE
//#define ELOG_ASYNC_OUTPUT_ENABLE
```

这套配置的特点是：

- 保留错误和警告
- 去掉信息、调试、详细日志
- 去掉颜色
- 去掉异步输出
- 不改业务主逻辑

它通常足以明显缩小日志系统 footprint。

## 19. 一个很重要的判断

单纯“优化 EasyLogger 本体”通常能省下一部分空间，但未必是最大头。  
按当前工程代码分布来看，真正更值得优先动的是：

- 大量 `printf`
- 大量浮点格式化
- 大量长字符串回显

所以对这次问题，最现实的认知应该是：

- EasyLogger 可以精简
- 但 Flash 超限更像是“整个日志体系过重”，不是“EasyLogger 单库过重”

## 20. 后续如果要真正落地

建议分两步做：

1. 先做“紧急瘦身版”
   - 目标：马上把链接过掉
   - 手段：关掉最重的 `printf` 和部分 `log_i/log_d`

2. 再做“发布版日志分层”
   - 调试版保留详细日志
   - 发布版只保留 `assert/error/warn`
   - 通过宏统一开关，而不是手工来回注释

这样后面维护成本最低，也不容易在排障和发布之间来回反复。
