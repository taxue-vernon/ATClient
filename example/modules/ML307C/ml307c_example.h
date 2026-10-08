/**
 * @file  ml307c_example.h
 * @brief ML307C 最小使用示例：静态创建 ATClient + ML307C，并通过 USART2/DMA 端口驱动。
 */

#ifndef ML307C_EXAMPLE_H
#define ML307C_EXAMPLE_H
#include "ML307C.h"
/**
 * @brief 初始化 ATClient、USART2 端口和 ML307C 驱动，并开始联网探测。
 * @param 无
 * @return true 全部初始化成功并已开始探测；false 任一步失败。
 * @note  示例不控制 EN/PWRKEY/DTR，调用前请先完成模组供电与开机。
 * @par 示例
 * @code
 * HAL_Init();
 * SystemClock_Config();
 * if (!ML307C_Example_Init()) {
 *     Error_Handler();
 * }
 * @endcode
 */
bool ML307C_Example_Init(void);

/**
 * @brief 示例主循环处理：依次调用 ATClient_Process、ML307C_Process、ML307C_UartPort_ProcessRaw。
 * @param 无
 * @return 无
 * @par 示例
 * @code
 * for (;;) {
 *     ML307C_Example_Process();
 * }
 * @endcode
 */
void ML307C_Example_Process(void);

/**
 * @brief 获取示例内部的 ML307C 对象，用于查询状态或发起 HTTP/短信（仅限主循环中使用）。
 * @param 无
 * @return 指向静态 ML307C 对象的指针。
 * @par 示例
 * @code
 * ML307C *modem = ML307C_Example_Modem();
 * if (ML307C_GetState(modem) == ML307C_STATE_NETWORK_READY) {
 *     (void)ML307C_HttpGet(modem, "/ping");
 * }
 * @endcode
 */
ML307C *ML307C_Example_Modem(void);
#endif
