/* 用户使用：先 GetTransport -> ATClient_Init -> UartPort_Init；
 * 主循环调用 ATClient_Process、Air780E_Process、UartPort_ProcessRaw。
 * 本文件已定义 HAL UART 回调与 USART2/DMA 句柄，已有定义时合并。
 * 只编译本文件或 ml307c_uart_port.c 之一；板级 MSP/IRQ 配置见本目录示例。 */

/**
 * @file  air780e_uart_port.c
 * @brief Air780E USART2/DMA 串口端口实现，并定义 huart2、hdma_usart2_rx/tx 句柄与 HAL UART 回调。
 *        公开接口的形参说明与示例见 air780e_uart_port.h。
 */

#include "air780e_uart_port.h"

#include <stddef.h>
#include <string.h>

#define AIR780E_UART_RAW_CHUNK_CAPACITY 32U

/** @brief 原始数据跟踪环形缓冲（ISR 写 head，主循环写 tail，单生产者单消费者）。 */
typedef struct
{
    uint8_t data[AIR780E_UART_RAW_CAPACITY];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t dropped;
    uint32_t reported_dropped;
} Air780E_UartRawRing;

/** @brief 端口内部状态：ATClient 指针、DMA 接收缓冲与游标、发送忙标志及跟踪缓冲。 */
typedef struct
{
    ATClient *client;
    uint8_t rx_dma_buffer[AIR780E_UART_RX_DMA_CAPACITY];
    uint16_t rx_cursor;
    bool tx_busy;
    Air780E_UartRawCallback raw_callback;
    void *raw_context;
    Air780E_UartRawRing raw_tx;
    Air780E_UartRawRing raw_rx;
} Air780E_UartPortState;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

static Air780E_UartPortState port_state;

static bool Air780E_UartPort_StartTx(void *context, const uint8_t *data, size_t length);
static uint32_t Air780E_UartPort_GetTimeMs(void *context);
static void Air780E_UartPort_PushSpan(uint16_t start, uint16_t end);
static void Air780E_UartPort_RawPush(Air780E_UartRawRing *ring, const uint8_t *data, size_t length);
static void Air780E_UartPort_RawDrain(Air780E_UartRawRing *ring,
                                      Air780E_UartRawDirection direction);

/**
 * @brief 获取供 ATClient 使用的传输接口（DMA 发送 + HAL_GetTick 时钟）。
 * @param 无
 * @return 传输接口结构体（按值返回）。
 * @par 示例
 * @code
 * ATClient_Transport transport = Air780E_UartPort_GetTransport();
 * (void)ATClient_Init(&at, &transport);
 * @endcode
 */
ATClient_Transport Air780E_UartPort_GetTransport(void)
{
    ATClient_Transport transport = {
        .start_tx = Air780E_UartPort_StartTx,
        .get_time_ms = Air780E_UartPort_GetTimeMs,
        .context = &port_state,
    };

    return transport;
}

/**
 * @brief 初始化 USART2 为 115200 8N1，并启动 DMA 循环接收（ReceiveToIdle）。
 * @param client 传已用 ATClient_Init() 初始化的 ATClient 地址（如 &at），收到的数据会推入其中。
 * @return true 成功；false client 为 NULL 或 HAL 初始化/启动接收失败。
 * @note  GPIO/DMA/NVIC 的底层配置在 HAL_UART_MspInit() 中完成。
 * @par 示例
 * @code
 * ATClient_Transport transport = Air780E_UartPort_GetTransport();
 * if (!ATClient_Init(&at, &transport) || !Air780E_UartPort_Init(&at)) {
 *     Error_Handler();
 * }
 * @endcode
 */
bool Air780E_UartPort_Init(ATClient *client)
{
    if (client == NULL) {
        return false;
    }

    memset(&port_state, 0, sizeof(port_state));
    memset(&huart2, 0, sizeof(huart2));
    memset(&hdma_usart2_rx, 0, sizeof(hdma_usart2_rx));
    memset(&hdma_usart2_tx, 0, sizeof(hdma_usart2_tx));
    port_state.client = client;

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200U;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart2) != HAL_OK) {
        return false;
    }

    /* Non-blocking receive: DMA owns the circular buffer. ISR callbacks only
     * forward newly written bytes into ATClient; parsing stays in main. */
    return HAL_UARTEx_ReceiveToIdle_DMA(&huart2, port_state.rx_dma_buffer,
                                        AIR780E_UART_RX_DMA_CAPACITY) == HAL_OK;
}

/**
 * @brief 设置（或清除）原始收发数据跟踪回调，用于串口日志调试。
 * @param callback 传打印收发数据的函数（类型见 Air780E_UartRawCallback）；传 NULL 关闭跟踪输出。
 * @param context  传任意指针，回调时原样交回；不需要时传 NULL。
 * @return 无
 * @par 示例
 * @code
 * static void trace(void *ctx, Air780E_UartRawDirection dir, const uint8_t *data,
 *                   size_t length, uint32_t dropped)
 * {
 *     (void)ctx;
 *     (void)dropped;
 *     printf("%s %.*s", dir == AIR780E_UART_RAW_TX ? ">>" : "<<", (int)length, (const char *)data);
 * }
 *
 * Air780E_UartPort_SetRawCallback(trace, NULL);
 * @endcode
 */
void Air780E_UartPort_SetRawCallback(Air780E_UartRawCallback callback, void *context)
{
    port_state.raw_callback = callback;
    port_state.raw_context = context;
}

/**
 * @brief 主循环处理：把跟踪缓冲中的收发数据交给跟踪回调输出。
 * @param 无
 * @return 无
 * @par 示例
 * @code
 * for (;;) {
 *     ATClient_Process(&at);
 *     Air780E_Process(&modem);
 *     Air780E_UartPort_ProcessRaw();
 * }
 * @endcode
 */
void Air780E_UartPort_ProcessRaw(void)
{
    if (port_state.raw_callback == NULL) {
        return;
    }

    Air780E_UartPort_RawDrain(&port_state.raw_tx, AIR780E_UART_RAW_TX);
    Air780E_UartPort_RawDrain(&port_state.raw_rx, AIR780E_UART_RAW_RX);
}

/**
 * @brief DMA 接收事件处理（半满/全满/空闲），把新到的字节区间推入 ATClient。
 * @param huart    传 HAL 回调收到的 huart 参数原样转交；不是 &huart2 时函数直接返回。
 * @param position 传 HAL_UARTEx_RxEventCallback 收到的 Size，即 DMA 缓冲区当前写入位置
 *                 （0..AIR780E_UART_RX_DMA_CAPACITY）。
 * @return 无
 * @par 示例
 * @code
 * void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
 * {
 *     Air780E_UartPort_RxEvent(huart, size);
 * }
 * @endcode
 */
void Air780E_UartPort_RxEvent(UART_HandleTypeDef *huart, uint16_t position)
{
    uint16_t previous;

    if (huart != &huart2 || port_state.client == NULL || position > AIR780E_UART_RX_DMA_CAPACITY) {
        return;
    }

    previous = port_state.rx_cursor;
    if (position == AIR780E_UART_RX_DMA_CAPACITY) {
        Air780E_UartPort_PushSpan(previous, AIR780E_UART_RX_DMA_CAPACITY);
        port_state.rx_cursor = 0U;
    } else if (position >= previous) {
        Air780E_UartPort_PushSpan(previous, position);
        port_state.rx_cursor = position;
    } else {
        Air780E_UartPort_PushSpan(previous, AIR780E_UART_RX_DMA_CAPACITY);
        Air780E_UartPort_PushSpan(0U, position);
        port_state.rx_cursor = position;
    }
}

/**
 * @brief DMA 发送完成处理：释放发送忙标志并通知 ATClient。
 * @param huart 传 HAL 回调收到的 huart 参数原样转交；不是 &huart2 时函数直接返回。
 * @return 无
 * @par 示例
 * @code
 * void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
 * {
 *     Air780E_UartPort_TxComplete(huart);
 * }
 * @endcode
 */
void Air780E_UartPort_TxComplete(UART_HandleTypeDef *huart)
{
    if (huart != &huart2 || port_state.client == NULL) {
        return;
    }

    port_state.tx_busy = false;
    ATClient_OnTxComplete(port_state.client);
}

/**
 * @brief 串口错误处理：释放发送忙标志，并通知 ATClient 当前事务失败。
 * @param huart 传 HAL 回调收到的 huart 参数原样转交；不是 &huart2 时函数直接返回。
 * @return 无
 * @par 示例
 * @code
 * void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 * {
 *     Air780E_UartPort_Error(huart);
 * }
 * @endcode
 */
void Air780E_UartPort_Error(UART_HandleTypeDef *huart)
{
    if (huart != &huart2 || port_state.client == NULL) {
        return;
    }

    port_state.tx_busy = false;
    ATClient_OnTransportError(port_state.client);
}

/**
 * @brief ATClient 传输层发送回调：启动一次 DMA 发送并立即返回。
 * @param context 由 ATClient 传入：传输接口里的 context，即 &port_state。
 * @param data    由 ATClient 传入：要发送的数据，发送完成前保持有效。
 * @param length  由 ATClient 传入：要发送的字节数，1..65535。
 * @return true 已启动 DMA；false 参数无效、正忙或 HAL 失败。
 * @par 示例
 * @code
 * ATClient_Transport transport = {
 *     .start_tx = Air780E_UartPort_StartTx,
 *     .get_time_ms = Air780E_UartPort_GetTimeMs,
 *     .context = &port_state,
 * };
 * @endcode
 */
static bool Air780E_UartPort_StartTx(void *context, const uint8_t *data, size_t length)
{
    Air780E_UartPortState *state = (Air780E_UartPortState *)context;

    if (state != &port_state || data == NULL || length == 0U || length > UINT16_MAX ||
        state->tx_busy) {
        return false;
    }

    /* Non-blocking: arm DMA exactly once and return. The HAL TX-complete
     * callback releases the port and advances ATClient later. */
    if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)data, (uint16_t)length) != HAL_OK) {
        return false;
    }
    Air780E_UartPort_RawPush(&state->raw_tx, data, length);
    state->tx_busy = true;
    return true;
}

/**
 * @brief ATClient 传输层时钟回调，返回 HAL_GetTick()。
 * @param context 由 ATClient 传入：传输接口里的 context（未使用）。
 * @return 当前毫秒计数。
 * @par 示例
 * @code
 * uint32_t now = Air780E_UartPort_GetTimeMs(NULL);
 * @endcode
 */
static uint32_t Air780E_UartPort_GetTimeMs(void *context)
{
    (void)context;
    return HAL_GetTick();
}

/**
 * @brief 把 DMA 接收缓冲中 [start, end) 区间的字节推入 ATClient 与跟踪缓冲。
 * @param start 传 DMA 接收缓冲区中新数据的起始下标（上次的接收游标）。
 * @param end   传新数据的结束下标（不含），即本次 DMA 写入位置。
 * @return 无
 * @par 示例
 * @code
 * Air780E_UartPort_PushSpan(previous, position);   // 未回绕
 * Air780E_UartPort_PushSpan(0U, position);         // 回绕后的前半段
 * @endcode
 */
static void Air780E_UartPort_PushSpan(uint16_t start, uint16_t end)
{
    if (end > start) {
        Air780E_UartPort_RawPush(&port_state.raw_rx, &port_state.rx_dma_buffer[start],
                                 (size_t)(end - start));
        (void)ATClient_PushRx(port_state.client, &port_state.rx_dma_buffer[start],
                              (size_t)(end - start));
    }
}

/**
 * @brief 向跟踪环形缓冲写入数据，空间不足的部分计入 dropped。
 * @param ring   传跟踪缓冲地址：发送用 &port_state.raw_tx，接收用 &port_state.raw_rx。
 * @param data   传要记录的数据首地址。
 * @param length 传 data 的字节数。
 * @return 无
 * @par 示例
 * @code
 * Air780E_UartPort_RawPush(&port_state.raw_tx, data, length);
 * @endcode
 */
static void Air780E_UartPort_RawPush(Air780E_UartRawRing *ring, const uint8_t *data, size_t length)
{
    uint32_t head = ring->head;
    uint32_t tail = ring->tail;
    size_t copied = 0U;

    while (copied < length && (uint32_t)(head - tail) < AIR780E_UART_RAW_CAPACITY) {
        ring->data[head % AIR780E_UART_RAW_CAPACITY] = data[copied];
        ++head;
        ++copied;
    }
    ring->head = head;
    ring->dropped += (uint32_t)(length - copied);
}

/**
 * @brief 从跟踪环形缓冲取出最多 32 字节，连同新增丢弃数一起交给跟踪回调。
 * @param ring      传要取数据的跟踪缓冲地址（&port_state.raw_tx 或 &port_state.raw_rx）。
 * @param direction 传与 ring 对应的方向 AIR780E_UART_RAW_TX / AIR780E_UART_RAW_RX，原样交给回调。
 * @return 无
 * @par 示例
 * @code
 * Air780E_UartPort_RawDrain(&port_state.raw_rx, AIR780E_UART_RAW_RX);
 * @endcode
 */
static void Air780E_UartPort_RawDrain(Air780E_UartRawRing *ring, Air780E_UartRawDirection direction)
{
    uint8_t chunk[AIR780E_UART_RAW_CHUNK_CAPACITY];
    uint32_t head = ring->head;
    uint32_t tail = ring->tail;
    uint32_t dropped = ring->dropped;
    uint32_t dropped_delta = dropped - ring->reported_dropped;
    size_t length = (size_t)(uint32_t)(head - tail);
    size_t index;

    if (length > AIR780E_UART_RAW_CHUNK_CAPACITY) {
        length = AIR780E_UART_RAW_CHUNK_CAPACITY;
    }
    for (index = 0U; index < length; ++index) {
        chunk[index] = ring->data[(tail + (uint32_t)index) % AIR780E_UART_RAW_CAPACITY];
    }
    ring->tail = tail + (uint32_t)length;
    ring->reported_dropped = dropped;

    if (length > 0U || dropped_delta > 0U) {
        port_state.raw_callback(port_state.raw_context, direction, chunk, length, dropped_delta);
    }
}

/**
 * @brief HAL 回调：DMA 接收事件（半满/全满/空闲），转交 Air780E_UartPort_RxEvent()。
 * @param huart 由 HAL 传入：触发接收事件的 UART 句柄。
 * @param size  由 HAL 传入：DMA 缓冲区当前写入位置。
 * @return 无
 * @par 示例
 * @code
 * // 由 HAL_UART_IRQHandler()/HAL_DMA_IRQHandler() 在中断中自动调用
 * @endcode
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    Air780E_UartPort_RxEvent(huart, size);
}

/**
 * @brief HAL 回调：DMA 发送完成，转交 Air780E_UartPort_TxComplete()。
 * @param huart 由 HAL 传入：发送完成的 UART 句柄。
 * @return 无
 * @par 示例
 * @code
 * // 由 HAL 在 DMA 发送完成中断中自动调用
 * @endcode
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    Air780E_UartPort_TxComplete(huart);
}

/**
 * @brief HAL 回调：串口错误（溢出、噪声、帧错误等），转交 Air780E_UartPort_Error()。
 * @param huart 由 HAL 传入：出错的 UART 句柄。
 * @return 无
 * @par 示例
 * @code
 * // 由 HAL_UART_IRQHandler() 检测到错误时自动调用
 * @endcode
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Air780E_UartPort_Error(huart);
}
