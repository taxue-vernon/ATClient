#ifndef AIR780E_UART_PORT_H
#define AIR780E_UART_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "ATClient.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AIR780E_UART_RX_DMA_CAPACITY  256U
#define AIR780E_UART_RAW_CAPACITY     512U

typedef enum
{
    AIR780E_UART_RAW_TX = 0,
    AIR780E_UART_RAW_RX
} Air780E_UartRawDirection;

typedef void (*Air780E_UartRawCallback)(void *context,
                                        Air780E_UartRawDirection direction,
                                        const uint8_t *data,
                                        size_t length,
                                        uint32_t dropped_bytes);

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

ATClient_Transport Air780E_UartPort_GetTransport(void);
bool Air780E_UartPort_Init(ATClient *client);
void Air780E_UartPort_SetRawCallback(Air780E_UartRawCallback callback,
                                     void *context);
void Air780E_UartPort_ProcessRaw(void);
void Air780E_UartPort_RxEvent(UART_HandleTypeDef *huart, uint16_t position);
void Air780E_UartPort_TxComplete(UART_HandleTypeDef *huart);
void Air780E_UartPort_Error(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
