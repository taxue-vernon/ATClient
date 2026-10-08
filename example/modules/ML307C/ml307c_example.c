/**
 * @file  ml307c_example.c
 * @brief ML307C 最小使用示例实现。接口说明与示例见 ml307c_example.h。
 */

#include "ml307c_example.h"
#include "ml307c_uart_port.h"

/* 用户使用步骤：HAL_Init -> 系统时钟 -> 模组供电稳定 -> Example_Init。
 * 在主循环不断调用 Example_Process；不要在中断中解析或打印。
 * 示例不控制 EN/PWRKEY/DTR，用户按所用底板完成供电与启动。 */
static ATClient client;
static ML307C modem;

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
bool ML307C_Example_Init(void)
{
    ATClient_Transport transport = ML307C_UartPort_GetTransport();
    if (!ATClient_Init(&client, &transport) || !ML307C_UartPort_Init(&client) ||
        !ML307C_Init(&modem, &client, NULL, NULL))
        return false;
    /* 把 NULL 换成 ML307C_Callbacks 和上下文可接收 HTTP/短信结果。
     * 默认只检测网络；如何配置 HTTP 和提交请求见本目录 README。 */
    return ML307C_Start(&modem) == ML307C_RESULT_OK;
}

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
void ML307C_Example_Process(void)
{
    ATClient_Process(&client);
    ML307C_Process(&modem);
    /* 即使没注册日志回调，也要调用：此函数还负责 UART 错误恢复。 */
    ML307C_UartPort_ProcessRaw();
}

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
ML307C *ML307C_Example_Modem(void)
{
    return &modem;
}
