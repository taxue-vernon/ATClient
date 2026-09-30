#include "air780e_example.h"
#include "air780e_uart_port.h"

/* 用户使用步骤：HAL_Init -> 系统时钟 -> 模组供电稳定 -> Example_Init。
 * 随后在主循环不断调用 Example_Process，不能在中断里调用。
 * ATClient/模组对象放静态区，避免大对象占用栈；一次只初始化一个实例。 */
static ATClient client;
static Air780E modem;

bool Air780E_Example_Init(void)
{
    ATClient_Transport transport = Air780E_UartPort_GetTransport();
    if (!ATClient_Init(&client, &transport) ||
        !Air780E_UartPort_Init(&client) ||
        !Air780E_Init(&modem, &client, NULL, NULL))
        return false;
    /* 需要业务通知时，把 NULL 换成 Air780E_Callbacks 和用户上下文。
     * 默认只探测模组与网络；MQTT 配置和发送见本目录 README。 */
    return Air780E_Start(&modem) == AIR780E_RESULT_OK;
}

void Air780E_Example_Process(void)
{
    ATClient_Process(&client);
    Air780E_Process(&modem);
    Air780E_UartPort_ProcessRaw();
}

Air780E *Air780E_Example_Modem(void)
{
    /* 仅从主循环访问，用户可查询状态或提交 MQTT 操作。 */
    return &modem;
}
