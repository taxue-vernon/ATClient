/**
 * @file    ml307c_uart_port.h
 * @brief   ML307C 的 STM32F1 HAL 串口端口：USART2（115200 8N1）+ DMA 收发，作为 ATClient 传输层。
 * @details RX 使用 DMA 循环缓冲 + 空闲中断（ReceiveToIdle），中断里只把新字节推入 ATClient；
 *          TX 使用 DMA 一次性发送。另带收发原始数据的跟踪环形缓冲，可选输出调试日志。
 *          本端口实现了 HAL_UARTEx_RxEventCallback / HAL_UART_TxCpltCallback /
 *          HAL_UART_ErrorCallback / HAL_UART_AbortCpltCallback，工程中不能再重复定义。
 *          初始化顺序：GetTransport -> ATClient_Init -> ML307C_UartPort_Init。
 */

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

#define ML307C_UART_RX_DMA_CAPACITY 256U
#define ML307C_UART_RAW_CAPACITY    512U

/** @brief 原始数据跟踪的方向：TX（MCU 发往模组）或 RX（模组发往 MCU）。 */
typedef enum
{
    ML307C_UART_RAW_TX = 0,
    ML307C_UART_RAW_RX
} ML307C_UartRawDirection;

/**
 * @brief 原始数据跟踪回调，在 ML307C_UartPort_ProcessRaw() 中（主循环上下文）调用。
 * @param context       由端口传入：ML307C_UartPort_SetRawCallback() 时给的 context。
 * @param direction     由端口传入：ML307C_UART_RAW_TX（MCU 发出）或
 *                      ML307C_UART_RAW_RX（模组发来）。
 * @param data          由端口传入：本次数据块（最多 32 字节），只在回调期间有效。
 * @param length        由端口传入：data 的字节数，可能为 0（仅报告丢弃）。
 * @param dropped_bytes 由端口传入：自上次回调以来因跟踪缓冲满而丢弃的字节数。
 */
typedef void (*ML307C_UartRawCallback)(void *context, ML307C_UartRawDirection direction,
                                       const uint8_t *data, size_t length, uint32_t dropped_bytes);

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/**
 * @brief 获取供 ATClient 使用的传输接口（DMA 发送 + HAL_GetTick 时钟）。
 * @param 无
 * @return 传输接口结构体（按值返回）。
 * @par 示例
 * @code
 * ATClient_Transport transport = ML307C_UartPort_GetTransport();
 * (void)ATClient_Init(&at, &transport);
 * @endcode
 */
ATClient_Transport ML307C_UartPort_GetTransport(void);

/**
 * @brief 初始化 USART2 为 115200 8N1，并启动 DMA 循环接收（ReceiveToIdle）。
 * @param client 传已用 ATClient_Init() 初始化的 ATClient 地址（如 &at），收到的数据会推入其中。
 * @return true 成功；false client 为 NULL 或 HAL 初始化/启动接收失败。
 * @note  GPIO/DMA/NVIC 的底层配置在 HAL_UART_MspInit() 中完成。
 * @par 示例
 * @code
 * ATClient_Transport transport = ML307C_UartPort_GetTransport();
 * if (!ATClient_Init(&at, &transport) || !ML307C_UartPort_Init(&at)) {
 *     Error_Handler();
 * }
 * @endcode
 */
bool ML307C_UartPort_Init(ATClient *client);

/* Stop DMA/UART and leave PA2/PA3 analog before EN cuts module power. */
/**
 * @brief 停止 USART2/DMA，并把 PA2/PA3 设为模拟输入，避免模组断电时 IO 倒灌。
 * @param 无
 * @return 无
 * @note  在通过 EN 关断模组电源之前调用；之后需重新 Init 才能通信。
 * @par 示例
 * @code
 * ML307C_UartPort_Shutdown();
 * CabinetUI_SetModemPower(false);
 * @endcode
 */
void ML307C_UartPort_Shutdown(void);

/**
 * @brief 设置（或清除）原始收发数据跟踪回调，用于串口日志调试。
 * @param callback 传打印收发数据的函数（类型见 ML307C_UartRawCallback）；传 NULL 关闭跟踪输出。
 * @param context  传任意指针，回调时原样交回；不需要时传 NULL。
 * @return 无
 * @par 示例
 * @code
 * static void trace(void *ctx, ML307C_UartRawDirection dir, const uint8_t *data,
 *                   size_t length, uint32_t dropped)
 * {
 *     (void)ctx;
 *     (void)dropped;
 *     printf("%s %.*s", dir == ML307C_UART_RAW_TX ? ">>" : "<<", (int)length, (const char *)data);
 * }
 *
 * ML307C_UartPort_SetRawCallback(trace, NULL);
 * @endcode
 */
void ML307C_UartPort_SetRawCallback(ML307C_UartRawCallback callback, void *context);

/**
 * @brief 暂停/恢复原始数据跟踪（切换时清空跟踪缓冲），不影响 AT 协议收发。
 * @param suppressed 传 true 暂停跟踪（如发送含 API Key 的 HTTP 正文前），传 false 恢复。
 * @return 无
 * @par 示例
 * @code
 * ML307C_UartPort_SetRawSuppressed(true);    // 发送敏感数据前
 * (void)ML307C_HttpPost(&modem, path, type, body, length);
 * // ... on_http_complete 回调中：
 * ML307C_UartPort_SetRawSuppressed(false);
 * @endcode
 */
void ML307C_UartPort_SetRawSuppressed(bool suppressed);

/**
 * @brief 主循环处理：执行串口错误恢复（Abort 后重新启动 DMA 接收），并输出跟踪数据。
 * @param 无
 * @return 无
 * @note  即使没有设置跟踪回调也必须周期调用，否则串口出错后无法恢复接收。
 * @par 示例
 * @code
 * for (;;) {
 *     ATClient_Process(&at);
 *     ML307C_Process(&modem);
 *     ML307C_UartPort_ProcessRaw();
 * }
 * @endcode
 */
void ML307C_UartPort_ProcessRaw(void);

/**
 * @brief DMA 接收事件处理（半满/全满/空闲），把新到的字节区间推入 ATClient。
 * @param huart    传 HAL 回调收到的 huart 参数原样转交；不是 &huart2 时函数直接返回。
 * @param position 传 HAL_UARTEx_RxEventCallback 收到的 Size，即 DMA 缓冲区当前写入位置
 *                 （0..ML307C_UART_RX_DMA_CAPACITY）。
 * @return 无
 * @par 示例
 * @code
 * void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
 * {
 *     ML307C_UartPort_RxEvent(huart, size);
 * }
 * @endcode
 */
void ML307C_UartPort_RxEvent(UART_HandleTypeDef *huart, uint16_t position);

/**
 * @brief DMA 发送完成处理：释放发送忙标志并通知 ATClient。
 * @param huart 传 HAL 回调收到的 huart 参数原样转交；不是 &huart2 时函数直接返回。
 * @return 无
 * @par 示例
 * @code
 * void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
 * {
 *     ML307C_UartPort_TxComplete(huart);
 * }
 * @endcode
 */
void ML307C_UartPort_TxComplete(UART_HandleTypeDef *huart);

/**
 * @brief 串口错误处理：请求在主循环中恢复接收，并通知 ATClient 当前事务失败。
 * @param huart 传 HAL 回调收到的 huart 参数原样转交；不是 &huart2 时函数直接返回。
 * @return 无
 * @par 示例
 * @code
 * void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 * {
 *     ML307C_UartPort_Error(huart);
 * }
 * @endcode
 */
void ML307C_UartPort_Error(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
