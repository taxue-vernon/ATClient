/* 用户使用：先 GetTransport -> ATClient_Init -> UartPort_Init；
 * 主循环调用 ATClient_Process、ML307C_Process、UartPort_ProcessRaw。
 * ProcessRaw 也执行串口恢复，即使不输出日志也必须调用。
 * 本文件已定义 HAL UART 回调与 USART2/DMA 句柄，已有定义时合并。
 * 只编译本文件或 air780e_uart_port.c 之一；板级 MSP/IRQ 配置见本目录示例。 */
#include "ml307c_uart_port.h"

#include <stddef.h>
#include <string.h>

#define ML307C_UART_RAW_CHUNK_CAPACITY  32U

typedef struct
{
    uint8_t data[ML307C_UART_RAW_CAPACITY];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t dropped;
    uint32_t reported_dropped;
} ML307C_UartRawRing;

typedef struct
{
    ATClient *client;
    uint8_t rx_dma_buffer[ML307C_UART_RX_DMA_CAPACITY];
    uint16_t rx_cursor;
    volatile bool tx_busy;
    volatile bool recovery_requested;
    volatile bool aborting;
    volatile bool restart_rx;
    volatile bool raw_suppressed;
    ML307C_UartRawCallback raw_callback;
    void *raw_context;
    ML307C_UartRawRing raw_tx;
    ML307C_UartRawRing raw_rx;
} ML307C_UartPortState;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

static ML307C_UartPortState port_state;

static bool ML307C_UartPort_StartTx(void *context,
                                     const uint8_t *data,
                                     size_t length);
static uint32_t ML307C_UartPort_GetTimeMs(void *context);
static void ML307C_UartPort_PushSpan(uint16_t start, uint16_t end);
static void ML307C_UartPort_RawPush(ML307C_UartRawRing *ring,
                                     const uint8_t *data,
                                     size_t length);
static void ML307C_UartPort_RawDrain(ML307C_UartRawRing *ring,
                                      ML307C_UartRawDirection direction);

ATClient_Transport ML307C_UartPort_GetTransport(void)
{
    ATClient_Transport transport = {
        .start_tx = ML307C_UartPort_StartTx,
        .get_time_ms = ML307C_UartPort_GetTimeMs,
        .context = &port_state,
    };

    return transport;
}

bool ML307C_UartPort_Init(ATClient *client)
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
                                        ML307C_UART_RX_DMA_CAPACITY) == HAL_OK;
}

void ML307C_UartPort_Shutdown(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* Block callbacks and TX before HAL tears down DMA and USART2. */
    port_state.client = NULL;
    if (huart2.Instance == USART2)
    {
        (void)HAL_UART_DeInit(&huart2);
    }
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);
    memset(&port_state, 0, sizeof(port_state));
    memset(&huart2, 0, sizeof(huart2));
    memset(&hdma_usart2_rx, 0, sizeof(hdma_usart2_rx));
    memset(&hdma_usart2_tx, 0, sizeof(hdma_usart2_tx));
}

void ML307C_UartPort_SetRawCallback(ML307C_UartRawCallback callback,
                                     void *context)
{
    port_state.raw_callback = callback;
    port_state.raw_context = context;
}

void ML307C_UartPort_SetRawSuppressed(bool suppressed)
{
    /* Set the gate before clearing on entry; clear before opening on exit.
     * UART protocol delivery is unaffected. */
    if (suppressed)
    {
        port_state.raw_suppressed = true;
    }
    memset(&port_state.raw_tx, 0, sizeof(port_state.raw_tx));
    memset(&port_state.raw_rx, 0, sizeof(port_state.raw_rx));
    if (!suppressed)
    {
        port_state.raw_suppressed = false;
    }
}

void ML307C_UartPort_ProcessRaw(void)
{
    /* HAL may stop circular RX after an overrun. Abort/rearm is driven here,
     * not in the ISR; Abort_IT returns without waiting for a UART byte. */
    if (port_state.recovery_requested && !port_state.aborting)
    {
        port_state.aborting = true;
        if (HAL_UART_Abort_IT(&huart2) != HAL_OK)
        {
            port_state.aborting = false;
        }
    }
    if (port_state.restart_rx)
    {
        port_state.rx_cursor = 0U;
        if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, port_state.rx_dma_buffer,
                                       ML307C_UART_RX_DMA_CAPACITY) == HAL_OK)
        {
            port_state.restart_rx = false;
        }
    }
    if (port_state.raw_callback == NULL || port_state.raw_suppressed)
    {
        return;
    }

    ML307C_UartPort_RawDrain(&port_state.raw_tx, ML307C_UART_RAW_TX);
    ML307C_UartPort_RawDrain(&port_state.raw_rx, ML307C_UART_RAW_RX);
}

void ML307C_UartPort_RxEvent(UART_HandleTypeDef *huart, uint16_t position)
{
    uint16_t previous;

    if (huart != &huart2 || port_state.client == NULL ||
        position > ML307C_UART_RX_DMA_CAPACITY)
    {
        return;
    }

    previous = port_state.rx_cursor;
    if (position == ML307C_UART_RX_DMA_CAPACITY)
    {
        ML307C_UartPort_PushSpan(previous, ML307C_UART_RX_DMA_CAPACITY);
        /* Keep the full cursor so TC + IDLE for the same boundary are not
         * forwarded twice. The next smaller position identifies the wrap. */
        port_state.rx_cursor = ML307C_UART_RX_DMA_CAPACITY;
    }
    else if (position >= previous)
    {
        ML307C_UartPort_PushSpan(previous, position);
        port_state.rx_cursor = position;
    }
    else
    {
        ML307C_UartPort_PushSpan(previous, ML307C_UART_RX_DMA_CAPACITY);
        ML307C_UartPort_PushSpan(0U, position);
        port_state.rx_cursor = position;
    }
}

void ML307C_UartPort_TxComplete(UART_HandleTypeDef *huart)
{
    if (huart != &huart2 || port_state.client == NULL)
    {
        return;
    }

    port_state.tx_busy = false;
    ATClient_OnTxComplete(port_state.client);
}

void ML307C_UartPort_Error(UART_HandleTypeDef *huart)
{
    if (huart != &huart2 || port_state.client == NULL)
    {
        return;
    }

    port_state.tx_busy = false;
    port_state.recovery_requested = true;
    ATClient_OnTransportError(port_state.client);
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        port_state.tx_busy = false;
        port_state.recovery_requested = false;
        port_state.aborting = false;
        port_state.restart_rx = true;
    }
}

static bool ML307C_UartPort_StartTx(void *context,
                                     const uint8_t *data,
                                     size_t length)
{
    ML307C_UartPortState *state = (ML307C_UartPortState *)context;

    if (state != &port_state || state->client == NULL ||
        huart2.Instance != USART2 || data == NULL || length == 0U ||
        length > UINT16_MAX || state->tx_busy || state->recovery_requested ||
        state->aborting || state->restart_rx)
    {
        return false;
    }

    /* Non-blocking: arm DMA exactly once and return. The HAL TX-complete
     * callback releases the port and advances ATClient later. */
    state->tx_busy = true;
    if (HAL_UART_Transmit_DMA(&huart2, (uint8_t *)data, (uint16_t)length) !=
        HAL_OK)
    {
        state->tx_busy = false;
        return false;
    }
    if (!state->raw_suppressed)
    {
        ML307C_UartPort_RawPush(&state->raw_tx, data, length);
    }
    return true;
}

static uint32_t ML307C_UartPort_GetTimeMs(void *context)
{
    (void)context;
    return HAL_GetTick();
}

static void ML307C_UartPort_PushSpan(uint16_t start, uint16_t end)
{
    if (end > start)
    {
        if (!port_state.raw_suppressed)
        {
            ML307C_UartPort_RawPush(&port_state.raw_rx,
                                    &port_state.rx_dma_buffer[start],
                                    (size_t)(end - start));
        }
        if (ATClient_PushRx(port_state.client,
                           &port_state.rx_dma_buffer[start],
                           (size_t)(end - start)) != ATCLIENT_RESULT_OK)
        {
            /* Protocol loss fails the current transaction; trace-ring loss
             * alone remains diagnostic and never takes this path. */
            ATClient_OnTransportError(port_state.client);
        }
    }
}

static void ML307C_UartPort_RawPush(ML307C_UartRawRing *ring,
                                     const uint8_t *data,
                                     size_t length)
{
    uint32_t head = ring->head;
    uint32_t tail = ring->tail;
    size_t copied = 0U;

    while (copied < length &&
           (uint32_t)(head - tail) < ML307C_UART_RAW_CAPACITY)
    {
        ring->data[head % ML307C_UART_RAW_CAPACITY] = data[copied];
        ++head;
        ++copied;
    }
    ring->head = head;
    ring->dropped += (uint32_t)(length - copied);
}

static void ML307C_UartPort_RawDrain(ML307C_UartRawRing *ring,
                                      ML307C_UartRawDirection direction)
{
    uint8_t chunk[ML307C_UART_RAW_CHUNK_CAPACITY];
    uint32_t head = ring->head;
    uint32_t tail = ring->tail;
    uint32_t dropped = ring->dropped;
    uint32_t dropped_delta = dropped - ring->reported_dropped;
    size_t length = (size_t)(uint32_t)(head - tail);
    size_t index;

    if (length > ML307C_UART_RAW_CHUNK_CAPACITY)
    {
        length = ML307C_UART_RAW_CHUNK_CAPACITY;
    }
    for (index = 0U; index < length; ++index)
    {
        chunk[index] = ring->data[(tail + (uint32_t)index) %
                                  ML307C_UART_RAW_CAPACITY];
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
    ML307C_UartPort_RxEvent(huart, size);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    ML307C_UartPort_TxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    ML307C_UartPort_Error(huart);
}
