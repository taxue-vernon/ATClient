/**
 * @file  stm32f1_uart_msp.c
 * @brief 移植示例：USART2（PA2=TX，PA3=RX）的 GPIO、DMA（RX 循环/TX 普通）与 NVIC 底层初始化。
 */

#include "stm32f1xx_hal.h"

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/* 用户移植：仅包含 USART2 的 MSP 初始化，已有同名函数时合并分支。 */
/**
 * @brief HAL 回调：USART2 初始化时配置时钟、引脚、DMA 通道 6/7 并使能中断。
 * @param huart 由 HAL_UART_Init()/HAL_UART_DeInit() 传入：正在初始化的 UART 句柄；只处理 USART2，
 *              其他实例直接返回。
 * @return 无
 * @par 示例
 * @code
 * // 由 HAL_UART_Init() 内部自动调用
 * huart2.Instance = USART2;
 * huart2.Init.BaudRate = 115200U;
 * (void)HAL_UART_Init(&huart2);
 * @endcode
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef gpio_init = {0};

    if (huart == NULL) {
        return;
    }

    if (huart->Instance == USART2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        gpio_init.Pin = GPIO_PIN_2;
        gpio_init.Mode = GPIO_MODE_AF_PP;
        gpio_init.Pull = GPIO_NOPULL;
        gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &gpio_init);

        gpio_init.Pin = GPIO_PIN_3;
        gpio_init.Mode = GPIO_MODE_INPUT;
        gpio_init.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &gpio_init);

        hdma_usart2_rx.Instance = DMA1_Channel6;
        hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
        (void)HAL_DMA_Init(&hdma_usart2_rx);
        __HAL_LINKDMA(huart, hdmarx, hdma_usart2_rx);

        hdma_usart2_tx.Instance = DMA1_Channel7;
        hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_tx.Init.Mode = DMA_NORMAL;
        hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
        (void)HAL_DMA_Init(&hdma_usart2_tx);
        __HAL_LINKDMA(huart, hdmatx, hdma_usart2_tx);

        HAL_NVIC_SetPriority(USART2_IRQn, 1U, 0U);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
        HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 1U, 1U);
        HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
        HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 1U, 1U);
        HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
        return;
    }
}

/**
 * @brief HAL 回调：USART2 反初始化时关闭时钟、释放引脚与 DMA、关闭中断。
 * @param huart 由 HAL_UART_Init()/HAL_UART_DeInit() 传入：正在初始化的 UART 句柄；只处理 USART2，
 *              其他实例直接返回。
 * @return 无
 * @par 示例
 * @code
 * // 由 HAL_UART_DeInit() 内部自动调用
 * (void)HAL_UART_DeInit(&huart2);
 * @endcode
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
    if (huart == NULL || huart->Instance != USART2) {
        return;
    }

    __HAL_RCC_USART2_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
    (void)HAL_DMA_DeInit(&hdma_usart2_rx);
    (void)HAL_DMA_DeInit(&hdma_usart2_tx);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel6_IRQn);
    HAL_NVIC_DisableIRQ(DMA1_Channel7_IRQn);
}
