# UART 适配层示例与移植指南

这里是 `example` 的第二部分。`stm32f1_hal/` 提供 STM32F103 的两种模组适配层：

| 文件 | 作用 |
| --- | --- |
| `air780e_uart_port.c/.h` | Air780E 的 USART2/DMA 适配与原始字节镜像 |
| `ml307c_uart_port.c/.h` | ML307C 适配、镜像屏蔽、错误恢复与关闭 |
| `stm32f1_uart_msp.c` | USART2 时钟、GPIO、DMA 和 NVIC 初始化/反初始化 |
| `stm32f1_uart_irq.c` | USART2 和 DMA1 Channel6/7 中断转发 |

两种适配层都定义 `huart2`、DMA 句柄及 HAL UART 回调，**只能选择一种编译**。
这些命名是既有工程约定，ATClient 本身不区分模组。

## STM32F103 HAL 接入

1. 添加所选 `*_uart_port.c`、MSP 和 IRQ 文件，头文件路径包含本目录的
   `stm32f1_hal/` 和 ATClient `src/`。
2. 需要支持 `HAL_UARTEx_ReceiveToIdle_DMA()` 的 STM32F1 HAL；启用 UART、DMA、GPIO
   HAL 模块，配置系统时钟和正常递增的 HAL tick。
3. PA2/TX 接模组 RX，PA3/RX 接模组 TX，共地；115200、8N1、无流控。
   核对实际底板 UART 电平和供电，必要时做电平转换。
4. RX 为 DMA1 Channel6、循环模式、256 字节，TX 为 Channel7、普通模式。
   保留 RX 半满、满及 UART IDLE 通知，不要禁用 HT 中断。
5. 初始化顺序为 `GetTransport -> ATClient_Init -> UartPort_Init -> 模组 Init/Start`。
   主循环持续运行 ATClient、模组 Process 和适配层 ProcessRaw。

**与 CubeMX/已有工程合并：**适配层自己定义 UART/DMA 句柄并调用 `HAL_UART_Init()`，
不要同时保留另一套同名句柄定义，也不要再调用另一套 USART2 初始化覆盖它。
已有 `HAL_UART_MspInit/DeInit` 时合并 USART2 分支；已有 IRQ 函数时合并 HAL 转发调用。
已有 HAL 回调时把适配层尾部的同名回调合并进唯一入口，按 UART 句柄分发；
ML307C 还需合并 `HAL_UART_AbortCpltCallback`。
SysTick 由用户工程维护，必须使 `HAL_GetTick()` 每毫秒递增。

原始日志不是必需的。需要时在 UartPort 初始化后调用 `SetRawCallback()`；回调在
`ProcessRaw()` 中执行，可按长度转义字节，并记录 `dropped_bytes`。
镜像丢字节仅影响日志；协议 RX 溢出则影响 AT 事务。
发送私有凭据时关闭镜像；ML307C 提供 `SetRawSuppressed()`。
Air780E 适配层只报告传输错误，UART 出错后的 DMA 中止/重启需要用户补充恢复策略。

## 移植到其他 UART 平台

只需提供下面两个函数并填入 transport，无需使用这里的 HAL 文件：

```c
/* 用户实现：启动异步发送，成功返回 true，立即返回，不等待 TX 完成。
 * 保持 data 指向的存储有效，完成事件只能通知一次。 */
bool user_start_tx(void *context, const uint8_t *data, size_t length);
/* 用户实现：返回单调递增的 uint32_t 毫秒计数，允许自然回绕。 */
uint32_t user_time_ms(void *context);

static ATClient client;
ATClient_Transport transport = { user_start_tx, user_time_ms, NULL };
/* 用户初始化代码中调用并检查返回值： */
bool initialized = ATClient_Init(&client, &transport);
```

- RX 中断/事件：把新字节传给 `ATClient_PushRx()` 并检查返回码。
  DMA 参数是写入位置时需像本示例一样计算新增区间，不能重复送入整个缓冲。
- TX 完成：调用 `ATClient_OnTxComplete()`。
- 传输失败：调用 `ATClient_OnTransportError()`，平台负责中止旧传输并恢复接收。
- 主循环/唯一处理任务：调用 `ATClient_Process()`。不要在 RX 中断里解析、打印或跑业务。
- RTOS 下保证单一 RX 生产者与单一解析消费者，Submit/注册接口由同一任务管理或外部串行化；
  当前库不内置锁，多核/缓存平台自行处理内存可见性、DMA 缓存一致性和同步。
