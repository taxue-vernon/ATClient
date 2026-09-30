#include "ml307c_example.h"
#include "ml307c_uart_port.h"

/* 用户使用步骤：HAL_Init -> 系统时钟 -> 模组供电稳定 -> Example_Init。
 * 在主循环不断调用 Example_Process；不要在中断中解析或打印。
 * 示例不控制 EN/PWRKEY/DTR，用户按所用底板完成供电与启动。 */
static ATClient client;
static ML307C modem;

bool ML307C_Example_Init(void)
{
    ATClient_Transport transport = ML307C_UartPort_GetTransport();
    if (!ATClient_Init(&client, &transport) ||
        !ML307C_UartPort_Init(&client) ||
        !ML307C_Init(&modem, &client, NULL, NULL))
        return false;
    /* 把 NULL 换成 ML307C_Callbacks 和上下文可接收 HTTP/短信结果。
     * 默认只检测网络；如何配置 HTTP 和提交请求见本目录 README。 */
    return ML307C_Start(&modem) == ML307C_RESULT_OK;
}

void ML307C_Example_Process(void)
{
    ATClient_Process(&client);
    ML307C_Process(&modem);
    /* 即使没注册日志回调，也要调用：此函数还负责 UART 错误恢复。 */
    ML307C_UartPort_ProcessRaw();
}

ML307C *ML307C_Example_Modem(void)
{
    return &modem;
}
