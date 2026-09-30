/* 用户使用：先 GetTransport -> ATClient_Init -> UartPort_Init；
 * 主循环调用 ATClient_Process、Air780E_Process、UartPort_ProcessRaw。
 * 本文件已定义 HAL UART 回调与 USART2/DMA 句柄，已有定义时合并。
 * 只编译本文件或 ml307c_uart_port.c 之一；板级 MSP/IRQ 配置见本目录示例。 */
#include "air780e_uart_port.h"

#include <stddef.h>
#include <string.h>

#define AIR780E_UART_RAW_CHUNK_CAPACITY  32U

typedef struct
{
    uint8_t data[AIR780E_UART_RAW_CAPACITY];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t dropped;
    uint32_t reported_dropped;
} Air780E_UartRawRing;

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

static bool Air780E_UartPort_StartTx(void *context,
                                     const uint8_t *data,
                                     size_t length);
static uint32_t Air780E_UartPort_GetTimeMs(void *context);
static void Air780E_UartPort_PushSpan(uint16_t start, uint16_t end);
static void Air780E_UartPort_RawPush(Air780E_UartRawRing *ring,
                                     const uint8_t *data,
                                     size_t length);
static void Air780E_UartPort_RawDrain(Air780E_UartRawRing *ring,
                                      Air780E_UartRawDirection direction);

ATClient_Transport Air780E_UartPort_GetTransport(void)
{
    ATClient_Transport transport = {
        .start_tx = Air780E_UartPort_StartTx,
        .get_time_ms = Air780E_UartPort_GetTimeMs,
        .context = &port_state,
    };

    return transport;
}

bool Air780E_UartPort_Init(ATClient *client)
{
    if (client == NULL)
    {
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

    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        return false;
    }

    /* Non-blocking receive: DMA owns the circular buffer. ISR callbacks only
     * forward newly written bytes into ATClient; parsing stays in main. */
    return HAL_UARTEx_ReceiveToIdle_DMA(&huart2,
                                        port_state.rx_dma_buffer,
                                        AIR780E_UART_RX_DMA_CAPACITY) == HAL_OK;
}

void Air780E_UartPort_SetRawCallback(Air780E_UartRawCallback callback,
                                     void *context)
{
    port_state.raw_callback = callback;
    port_state.raw_context = context;
}

void Air780E_UartPort_ProcessRaw(void)
{
    if (port_state.raw_callback == NULL)
    {
        return;
    }

    Air780E_UartPort_RawDrain(&port_state.raw_tx, AIR780E_UART_RAW_TX);
    Air780E_UartPort_RawDrain(&port_state.raw_rx, AIR780E_UART_RAW_RX);
}

void Air780E_UartPort_RxEvent(UART_HandleTypeDef *huart, uint16_t position)
{
    uint16_t previous;

    if (huart != &huart2 || port_state.client == NULL ||
        position > AIR780E_UART_RX_DMA_CAPACITY)
    {
        return;
    }

    previous = port_state.rx_cursor;
    if (position == AIR780E_UART_RX_DMA_CAPACITY)
    {
        Air780E_UartPort_PushSpan(previous, AIR780E_UART_RX_DMA_CAPACITY);
        port_state.rx_cursor = 0U;
    }
    else if (position >= previous)
    {
        Air780E_UartPort_PushSpan(previous, position);
        port_state.rx_cursor = position;
    }
    else
    {
        Air780E_UartPort_PushSpan(previous, AIR780E_UART_RX_DMA_CAPACITY);
        Air780E_UartPort_PushSpan(0U, position);
        port_state.rx_cursor = position;
    }
}

void Air780E_UartPort_TxComplete(UART_HandleTypeDef *huart)
{
    if (huart != &huart2 || port_state.client == NULL)
    {
        return;
    }

    port_state.tx_busy = false;
    ATClient_OnTxComplete(port_state.client);
}

void Air780E_UartPort_Error(UART_HandleTypeDef *huart)
{
    if (huart != &huart2 || port_state.client == NULL)
    {
        return;
    }

    port_state.tx_busy = false;
    ATClient_OnTransportError(port_state.client);
}

static bool Air780E_UartPort_StartTx(void *context,
                                     const uint8_t *data,
                                     size_t length)
{
    Air780E_UartPortState *state = (Air780E_UartPortState *)context;

    if (state != &port_state || data == NULL || length == 0U ||
        length > UINT16_MAX || state->tx_busy)
    {
        return false;
    }

    /* Non-blocking: arm DMA exactly once and return. The HAL TX-complete
     * callback releases the port and advances ATClient later. */
    if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)data, (uint16_t)length) !=
        HAL_OK)
    {
        return false;
    }
    Air780E_UartPort_RawPush(&state->raw_tx, data, length);
    state->tx_busy = true;
    return true;
}

static uint32_t Air780E_UartPort_GetTimeMs(void *context)
{
    (void)context;
    return HAL_GetTick();
}

static void Air780E_UartPort_PushSpan(uint16_t start, uint16_t end)
{
    if (end > start)
    {
        Air780E_UartPort_RawPush(&port_state.raw_rx,
                                 &port_state.rx_dma_buffer[start],
                                 (size_t)(end - start));
        (void)ATClient_PushRx(port_state.client,
                              &port_state.rx_dma_buffer[start],
                              (size_t)(end - start));
    }
}

static void Air780E_UartPort_RawPush(Air780E_UartRawRing *ring,
                                     const uint8_t *data,
                                     size_t length)
{
    uint32_t head = ring->head;
    uint32_t tail = ring->tail;
    size_t copied = 0U;

    while (copied < length &&
           (uint32_t)(head - tail) < AIR780E_UART_RAW_CAPACITY)
    {
        ring->data[head % AIR780E_UART_RAW_CAPACITY] = data[copied];
        ++head;
        ++copied;
    }
    ring->head = head;
    ring->dropped += (uint32_t)(length - copied);
}

static void Air780E_UartPort_RawDrain(Air780E_UartRawRing *ring,
                                      Air780E_UartRawDirection direction)
{
    uint8_t chunk[AIR780E_UART_RAW_CHUNK_CAPACITY];
    uint32_t head = ring->head;
    uint32_t tail = ring->tail;
    uint32_t dropped = ring->dropped;
    uint32_t dropped_delta = dropped - ring->reported_dropped;
    size_t length = (size_t)(uint32_t)(head - tail);
    size_t index;

    if (length > AIR780E_UART_RAW_CHUNK_CAPACITY)
    {
        length = AIR780E_UART_RAW_CHUNK_CAPACITY;
    }
    for (index = 0U; index < length; ++index)
    {
        chunk[index] = ring->data[(tail + (uint32_t)index) %
                                  AIR780E_UART_RAW_CAPACITY];
    }
    ring->tail = tail + (uint32_t)length;
    ring->reported_dropped = dropped;

    if (length > 0U || dropped_delta > 0U)
    {
        port_state.raw_callback(port_state.raw_context,
                                direction,
                                chunk,
                                length,
                                dropped_delta);
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    Air780E_UartPort_RxEvent(huart, size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    Air780E_UartPort_TxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    Air780E_UartPort_Error(huart);
}
