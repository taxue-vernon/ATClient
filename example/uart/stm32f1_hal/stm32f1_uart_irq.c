#include "stm32f1xx_hal.h"
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/* 用户移植：若工程已有这些中断函数，把调用合并进去，不要重复定义。
 * SysTick 由用户工程提供，必须正常调用 HAL_IncTick()。
 * UART 的 HAL 完成/接收/错误回调已在所选 uart_port.c 中实现。 */
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
void DMA1_Channel6_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart2_rx); }
void DMA1_Channel7_IRQHandler(void) { HAL_DMA_IRQHandler(&hdma_usart2_tx); }
