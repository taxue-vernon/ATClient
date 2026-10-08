/**
 * @file  stm32f1_uart_irq.c
 * @brief 移植示例：USART2 及其 DMA1 通道 6/7 的中断入口，转交给 HAL 处理。
 */

#include "stm32f1xx_hal.h"
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/* 用户移植：若工程已有这些中断函数，把调用合并进去，不要重复定义。
 * SysTick 由用户工程提供，必须正常调用 HAL_IncTick()。
 * UART 的 HAL 完成/接收/错误回调已在所选 uart_port.c 中实现。 */
/**
 * @brief USART2 全局中断入口（空闲线、错误等），调用 HAL_UART_IRQHandler()。
 * @param 无
 * @return 无
 * @par 示例
 * @code
 * // 由中断向量表自动调用，用户无需手动调用；
 * // 若工程已有该函数，只需在其中加入：
 * HAL_UART_IRQHandler(&huart2);
 * @endcode
 */
void USART2_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart2);
}

/**
 * @brief DMA1 通道 6（USART2_RX）中断入口。
 * @param 无
 * @return 无
 * @par 示例
 * @code
 * // 由中断向量表自动调用；已有该函数时合并：
 * HAL_DMA_IRQHandler(&hdma_usart2_rx);
 * @endcode
 */
void DMA1_Channel6_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

/**
 * @brief DMA1 通道 7（USART2_TX）中断入口。
 * @param 无
 * @return 无
 * @par 示例
 * @code
 * // 由中断向量表自动调用；已有该函数时合并：
 * HAL_DMA_IRQHandler(&hdma_usart2_tx);
 * @endcode
 */
void DMA1_Channel7_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart2_tx);
}
