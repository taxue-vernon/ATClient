# ML307C 驱动示例

`src/ML307C.c/.h` 为完整驱动源码；`ml307c_example.c/.h` 展示初始化与主循环调度。
搭配 `../../uart/stm32f1_hal/ml307c_uart_port.c/.h`，编译接入按
[根 README](../../../README.md)进行。示例只检测网络，不自动发送 HTTP 或短信。

## 如何使用

1. 用户完成模组底板供电/启动后，调用 `ML307C_Example_Init()` 并检查返回值。
2. 主循环不断调用 `ML307C_Example_Process()`，其中 `ProcessRaw()` 还负责串口恢复。
3. 在示例的 `ML307C_Init()` 处传入 `ML307C_Callbacks` 和用户上下文，接收结果。
4. 初始化成功后配置一次 HTTP，网络就绪后按应用事件提交一次 GET/POST。

```c
/* 换成自己控制的测试服务地址；host 不含路径，路径由 Get/Post 单独传入。 */
static const ML307C_HttpConfig config = {
    .host = "http://192.0.2.1:8080", /* 文档用地址，必须替换为可访问的服务 */
    .read_chunk_size = 256,
};
bool configured = ML307C_HttpConfigure(ML307C_Example_Modem(), &config);
/* 检查 configured。网络达到 ML307C_STATE_NETWORK_READY 后提交一次： */
ML307C_Result result = ML307C_HttpGet(ML307C_Example_Modem(), "/status");
/* BUSY/NOT_READY 表示未接受；OK 表示接受，等待 on_http_complete。 */
```

HTTP 请求不自动重发，不要在每轮主循环重复提交。`on_http_response` 给出状态码；
`on_http_data` 分块交付响应头/正文，`data` 只在回调期间有效，使用 `length`，
不可按字符串打印。`on_http_complete` 表示流程完成，通信结果 OK 不代表 HTTP 2xx。
host 借用到下次成功配置；请求 path 和 POST body 必须保持到完成回调返回。

```c
static const uint8_t body[] = "{\"value\":1}";
/* 网络就绪且上次 HTTP 完成后，按用户事件调用一次并检查返回值。 */
ML307C_Result result = ML307C_HttpPost(ML307C_Example_Modem(),
    "/report", "application/json", body, sizeof(body) - 1);
```

HTTPS 的 host 使用 `https://`，必须同时提供有效 `ca_pem`、`ca_length` 和
版本化 `ca_name`；证书数据须保持到准备完成，等待 `tls_ready` 后再请求。
示例不携带应用账号、API Key 或服务器专用证书，按实际服务配置并验证固件 TLS 行为。

短信通过 `ML307C_SendSms(modem, number, text)` 提交，最终结果由
`on_sms_complete` 通知。当前只支持 1～160 字节可打印 ASCII，不支持中文或长短信，
要求数据网络就绪，不自动重发。HTTP 正文和响应规模、状态等限制详见驱动头文件与源码。
模组电源 EN/PWRKEY/DTR 与 GPIO 配置由用户工程负责。
