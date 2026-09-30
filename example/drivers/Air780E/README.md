# Air780E 驱动示例

`src/Air780E.c/.h` 为完整驱动源码；`air780e_example.c/.h` 展示初始化与主循环调度。
核心和驱动不依赖 HAL；本示例选择 `../../uart/stm32f1_hal/air780e_uart_port.c/.h`。
编译文件和板级准备按[根 README](../../../README.md)进行。

## 如何使用

1. 模组上电稳定后调用 `Air780E_Example_Init()` 并检查返回值。
2. 主循环持续调用 `Air780E_Example_Process()`。
3. 用 `Air780E_GetStatus(Air780E_Example_Modem())` 读取 SIM、注册和数据附着状态。
4. 需要 MQTT 时填写以下配置，初始化成功后配置一次。字符串须持续有效。

```c
static const Air780E_MqttConfig config = {
    .host = "你的 MQTT 服务器", /* 仅主机名/IP，不带 mqtt:// */
    .port = 1883,
    .client_id = "你的唯一客户端ID",
    .username = "", .password = "",
    .clean_session = true, .keepalive_s = 60, .auto_reconnect = true,
};
Air780E_Result result = Air780E_MqttConfigure(Air780E_Example_Modem(), &config);
/* 必须检查 result == AIR780E_RESULT_OK。私有配置不要提交到公开仓库。 */
```

网络检测完成且 `packet_attached` 为真后，在主循环提交一次
`Air780E_MqttConnect()`。`BUSY/NOT_READY` 表示未接受，之后根据状态重试；
返回 `OK` 只表示接受连接操作。等待 `mqtt_online` 为真再订阅或发布。
不要每次主循环无条件重复调用连接/发布接口。

```c
/* MQTT 在线且无其他 MQTT 操作时按应用事件调用一次。 */
static const uint8_t payload[] = "hello";
Air780E_Result result = Air780E_MqttPublish(
    Air780E_Example_Modem(), "demo/status", payload, sizeof(payload) - 1, 1, false);
/* 检查 result；发布的最终结果由 on_mqtt_operation_complete 通知。 */
```

在示例 `Air780E_Init()` 处把 NULL 替换为 `Air780E_Callbacks` 和用户上下文，
可接收启动结果、MQTT 操作结果与流式消息。消息数据仅在回调期间有效，按长度处理，
不能假设以 NUL 结束；回调短小且不阻塞。发布 topic/payload 与订阅 topic
在完成回调返回之前不得修改或释放。

此驱动提供 MQTT 3.1.1、QoS 0/1/2、单连接和缓存消息接收，未实现 TLS。
模组须运行兼容的 AT 固件；LuatOS 固件不能直接按此 AT 命令流程使用。
供电和 UART 电平由所用底板决定，先核对接口再接线。
