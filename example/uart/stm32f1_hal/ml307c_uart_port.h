#ifndef ML307C_UART_PORT_H
#define ML307C_UART_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "stm32f1xx_hal.h"

#include "ATClient.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ML307C_UART_RX_DMA_CAPACITY  256U
#define ML307C_UART_RAW_CAPACITY     512U

typedef enum
{
    ML307C_UART_RAW_TX = 0,
    ML307C_UART_RAW_RX
} ML307C_UartRawDirection;

typedef void (*ML307C_UartRawCallback)(void *context,
                                        ML307C_UartRawDirection direction,
                                        const uint8_t *data,
                                        size_t length,
                                        uint32_t dropped_bytes);

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

ATClient_Transport ML307C_UartPort_GetTransport(void);
bool ML307C_UartPort_Init(ATClient *client);
/* Stop DMA/UART and leave PA2/PA3 analog before EN cuts module power. */
void ML307C_UartPort_Shutdown(void);
void ML307C_UartPort_SetRawCallback(ML307C_UartRawCallback callback,
                                     void *context);
void ML307C_UartPort_SetRawSuppressed(bool suppressed);
void ML307C_UartPort_ProcessRaw(void);
void ML307C_UartPort_RxEvent(UART_HandleTypeDef *huart, uint16_t position);
void ML307C_UartPort_TxComplete(UART_HandleTypeDef *huart);
void ML307C_UartPort_Error(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
