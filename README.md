# ATClient

`ATClient` 是一个与 MCU、HAL 和具体模组命令集无关的异步 AT 客户端。
它只依赖调用者注入的 `start_tx()` 和 `get_time_ms()`，不包含 STM32 HAL，
不使用动态内存，也不会等待 UART、响应字符串或超时。

## 目录结构

```text
ATClient/
├── src/                          # 通用 AT 客户端，仅依赖标准 C 头文件
│   ├── ATClient.c
│   └── ATClient.h
├── example/
│   ├── modules/                  # 第一部分：上层模组协议源码和使用示例
│   │   ├── Air780E/              # 网络检测、MQTT
│   │   └── ML307C/               # 网络检测、HTTP/HTTPS、ASCII 短信
│   └── uart/                     # 第二部分：UART 适配层示例
│       └── stm32f1_hal/          # USART2 + RX 循环 DMA + TX DMA
└── docs/
    └── ATClient_Air780E设计思路讲解.html
```

示例包含所需模组协议源码，复制整个 ATClient 文件夹即可保留这些示例。
`example` 不属于核心库的默认编译范围；在用户工程中显式选择所需 `.c` 文件。
模组协议与 UART 示例来自 JPSmartCube 当前代码，板级初始化仅提取 USART2 部分。

`modules` 是 ATClient 的上层模组协议示例，负责网络、MQTT、HTTP 等业务流程；
UART 适配层位于 ATClient 下层，负责字节收发。模组协议层依赖 ATClient，
ATClient 通过注入的 transport 接口使用 UART 适配层。

## 快速接入

1. 把 `src/ATClient.c` 加入工程编译，并把 `src/` 加入头文件搜索路径。
2. 选择一种模组，将其 `example/modules/<模组>/src/` 和对应的
   `*_example.c` 加入编译；将这两个目录加入头文件搜索路径。
3. STM32F103 HAL 用户再添加 `example/uart/stm32f1_hal/` 头文件路径，
   编译对应的一组 `*_uart_port.c/.h`，以及 `stm32f1_uart_msp.c`、
   `stm32f1_uart_irq.c`。已有 MSP、中断、HAL 回调时按
   [UART 说明](example/uart/README.md)合并，避免重定义。
4. 完成 HAL、系统时钟、SysTick、模组供电初始化，然后调用示例初始化函数。
   初始化返回 `false` 时处理失败，不继续调度。
5. 在主循环持续调用示例的 `Process()`，通过状态接口与完成回调推进应用。

例如选择 ML307C：

```c
#include "ml307c_example.h"

/* 在用户 main() 中：HAL_Init()、SystemClock_Config() 和底板启动之后。 */
if (!ML307C_Example_Init()) {
    Error_Handler(); /* 用户提供初始化失败处理 */
}
for (;;) {
    ML307C_Example_Process();
    /* 在这里调度其他任务，查询状态并按需提交一次业务操作。 */
}
```

Air780E 对应使用 `air780e_example.h`、`Air780E_Example_Init()` 和
`Air780E_Example_Process()`。两套示例都使用 USART2 和同名 HAL 回调，
**同一工程只编译其中一套 UART 适配层**。
示例是集成片段，系统时钟、启动文件、HAL 库和板级电源由用户工程提供。

其他 MCU、LL、裸寄存器或 RTOS 平台只需实现 `ATClient_Transport`，
无需修改核心解析器。详见 [UART 适配指南](example/uart/README.md)。

## 示例说明与设计文档

- [Air780E 上层模组使用](example/modules/Air780E/README.md)
- [ML307C 上层模组使用](example/modules/ML307C/README.md)
- [UART 接入与移植](example/uart/README.md)
- [ATClient / Air780E 设计思路](docs/ATClient_Air780E设计思路讲解.html)

设计 HTML 从 JPSmartCube 的 `docs` 原样复制，可用浏览器离线打开。
文中的工程路径、测试统计和目标内存数据属于原文版本；当前接口以本目录源码为准。

## 调用模型

1. 创建 `ATClient_Transport`，其中 `start_tx()` 只启动异步发送并立即返回。
2. 调用 `ATClient_Init()`。
3. DMA/中断收到字节时只调用 `ATClient_PushRx()`。
4. DMA TX 完成时调用 `ATClient_OnTxComplete()`；传输错误时调用
   `ATClient_OnTransportError()`。
5. 在主循环高频调用 `ATClient_Process()`。

中断中不得调用 `ATClient_Process()`，也不得执行应用回调或格式化日志。
响应行、URC、事务完成和长度帧回调都在调用 `ATClient_Process()` 的上下文中
同步执行。本工程从主循环调用它，因此不会在 ISR 中运行用户代码。
RX 环形缓冲采用单生产者/单消费者计数器：ISR 只推进 head，主循环只推进
tail，避免双方同时修改共享计数值。

## 固定容量

- RX 环形缓冲：1024 字节。
- 普通行/长度帧头：384 字节。
- 活动命令：1024 字节，另有 2 字节 CRLF 空间。
- 待处理事务：4 条。
- URC 注册：8 个。
- 长度帧注册：4 个。
- 单个流式 payload 声明长度上限：4100 字节。

4100 字节 payload 不会整体复制到 ATClient。长度帧解析器在读取固定大小头部后，
按 RX 环形缓冲中当前已有的数据分块回调。

## 所有权

- `ATClient_Submit()` 会复制事务描述符，但不会复制描述符指向的 payload、命令
  上下文或回调上下文。
- 上述指针从提交成功起，到完成回调返回前必须保持有效。
- prompt payload 采用零拷贝发送，调用者不得在完成前释放或修改它。
- transport、URC 和长度帧注册中保存的上下文必须至少与 `ATClient` 同寿命。

普通 `OK` 与业务完成可以分离。事务可声明有序成功 token，例如
`OK -> CONNECT OK` 或 `OK -> PUBREC -> PUBCOMP`，也可在 `>` prompt 后发送
定长二进制 payload。
