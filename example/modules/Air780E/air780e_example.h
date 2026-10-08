/**
 * @file  air780e_example.h
 * @brief Air780E 最小使用示例：静态创建 ATClient + Air780E，并通过 USART2/DMA 端口驱动。
 */

#ifndef AIR780E_EXAMPLE_H
#define AIR780E_EXAMPLE_H
#include "Air780E.h"
/**
 * @brief 初始化 ATClient、USART2 端口和 Air780E 驱动，并开始启动探测。
 * @param 无
 * @return true 全部初始化成功并已开始探测；false 任一步失败。
 * @par 示例
 * @code
 * HAL_Init();
 * SystemClock_Config();
 * if (!Air780E_Example_Init()) {
 *     Error_Handler();
 * }
 * @endcode
 */
bool Air780E_Example_Init(void);

/**
 * @brief 示例主循环处理：依次调用 ATClient_Process、Air780E_Process、Air780E_UartPort_ProcessRaw。
 * @param 无
 * @return 无
 * @par 示例
 * @code
 * for (;;) {
 *     Air780E_Example_Process();
 * }
 * @endcode
 */
void Air780E_Example_Process(void);

/**
 * @brief 获取示例内部的 Air780E 对象，用于查询状态或提交 MQTT 操作（仅限主循环中使用）。
 * @param 无
 * @return 指向静态 Air780E 对象的指针。
 * @par 示例
 * @code
 * Air780E *modem = Air780E_Example_Modem();
 * if (Air780E_GetState(modem) == AIR780E_STATE_MQTT_ONLINE) {
 *     (void)Air780E_MqttSubscribe(modem, "cube/cmd", 0U);
 * }
 * @endcode
 */
Air780E *Air780E_Example_Modem(void);
#endif
