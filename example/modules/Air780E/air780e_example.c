/**
 * @file  air780e_example.c
 * @brief Air780E 最小使用示例实现。接口说明与示例见 air780e_example.h。
 */

#include "air780e_example.h"
#include "air780e_uart_port.h"

/* 用户使用步骤：HAL_Init -> 系统时钟 -> 模组供电稳定 -> Example_Init。
 * 随后在主循环不断调用 Example_Process，不能在中断里调用。
 * ATClient/模组对象放静态区，避免大对象占用栈；一次只初始化一个实例。 */
static ATClient client;
static Air780E modem;

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
bool Air780E_Example_Init(void)
{
    ATClient_Transport transport = Air780E_UartPort_GetTransport();
    if (!ATClient_Init(&client, &transport) || !Air780E_UartPort_Init(&client) ||
        !Air780E_Init(&modem, &client, NULL, NULL))
        return false;
    /* 需要业务通知时，把 NULL 换成 Air780E_Callbacks 和用户上下文。
     * 默认只探测模组与网络；MQTT 配置和发送见本目录 README。 */
    return Air780E_Start(&modem) == AIR780E_RESULT_OK;
}

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
void Air780E_Example_Process(void)
{
    ATClient_Process(&client);
    Air780E_Process(&modem);
    Air780E_UartPort_ProcessRaw();
}

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
Air780E *Air780E_Example_Modem(void)
{
    /* 仅从主循环访问，用户可查询状态或提交 MQTT 操作。 */
    return &modem;
}
