/**
 * @file    ML307C.h
 * @brief   中移物联 ML307C 4G 模组驱动：开机探测、驻网/PDP 激活、HTTP(S) 请求、短信与空闲睡眠。
 * @details 基于 ATClient 的协作式状态机，所有接口均非阻塞。典型流程：
 *          ATClient_Init() -> ML307C_Init() -> ML307C_Start()，
 *          随后主循环同时调用 ATClient_Process() 与 ML307C_Process()；
 *          网络就绪（ML307C_STATE_NETWORK_READY）后即可发起 HTTP 或短信。
 */

#ifndef ML307C_H
#define ML307C_H
#include "ATClient.h"
#ifdef __cplusplus
extern "C" {
#endif

/** @brief ML307C 接口与异步回调使用的结果码。 */
typedef enum
{
    ML307C_RESULT_OK,
    ML307C_RESULT_BUSY,
    ML307C_RESULT_INVALID_ARGUMENT,
    ML307C_RESULT_AT_ERROR,
    ML307C_RESULT_TIMEOUT,
    ML307C_RESULT_PARSE_ERROR,
    ML307C_RESULT_NOT_READY,
    ML307C_RESULT_NETWORK_LOST,
    ML307C_RESULT_HTTP_ERROR
} ML307C_Result;

/** @brief 模组对外可见的网络状态，变化时触发 on_state_changed。 */
typedef enum
{
    ML307C_STATE_OFFLINE,
    ML307C_STATE_PROBING,
    ML307C_STATE_SIM_WAIT,
    ML307C_STATE_NETWORK_WAIT,
    ML307C_STATE_DATA_ACTIVATING,
    ML307C_STATE_NETWORK_READY,
    ML307C_STATE_RETRY_WAIT,
    ML307C_STATE_ERROR
} ML307C_State;

/** @brief on_http_data 回调中数据块的类型：响应头或响应正文。 */
typedef enum
{
    ML307C_HTTP_HEADER,
    ML307C_HTTP_CONTENT
} ML307C_HttpDataType;

/** @brief HTTP(S) 服务器配置，由 ML307C_HttpConfigure() 复制保存（字符串只保存指针）。 */
typedef struct
{
    const char *host;         /* http(s)://authority[:port], borrowed */
    uint16_t read_chunk_size; /* 0 = 256; 1..1024 */
    const uint8_t *ca_pem;    /* HTTPS PEM, borrowed until prepared */
    size_t ca_length;
    const char *ca_name; /* versioned modem certificate name */
} ML307C_HttpConfig;

/** @brief 模组状态快照：型号/固件/IMEI、信号、注册、IP、HTTP 与短信统计等，只读。 */
typedef struct
{
    bool responsive, matready_seen, sim_ready, rssi_known, data_active, http_busy, sms_busy;
    bool tls_ready;
    char model[32], firmware[64], imei[16], ipv4[16], ipv6[46];
    uint8_t cfun, rssi, ber, registration;
    int16_t rssi_dbm;
    uint16_t http_status;
    uint32_t http_error, http_received_bytes, http_request_count, sms_reference;
    uint32_t retry_count, parse_error_count, urc_error_count, last_check_ms;
    ML307C_Result last_result, cleanup_result;
} ML307C_Status;

/**
 * @brief 异步事件回调集合，任一成员可为 NULL；均在 ML307C_Process()/ATClient_Process() 中调用。
 * @details 回调形参（第一个参数都是 ML307C_Init() 传入的 ctx）：
 *          - on_state_changed(ctx, state)：网络状态变化；
 *          - on_startup_complete(ctx, result)：首次进入网络就绪；
 *          - on_http_response(ctx, status, header_length, body_length)：收到 HTTP 状态行；
 *          - on_http_data(ctx, type, data, length)：读到响应头/正文数据块；
 *          - on_http_complete(ctx, result)：HTTP 请求结束（含清理）；
 *          - on_sms_complete(ctx, result, message_reference)：短信发送结束。
 */
typedef struct
{
    void (*on_state_changed)(void *, ML307C_State);
    void (*on_startup_complete)(void *, ML307C_Result);
    void (*on_http_response)(void *, uint16_t, size_t, size_t);
    void (*on_http_data)(void *, ML307C_HttpDataType, const uint8_t *, size_t);
    void (*on_http_complete)(void *, ML307C_Result);
    void (*on_sms_complete)(void *, ML307C_Result, uint32_t message_reference);
} ML307C_Callbacks;

/* Internal cooperative stages; applications use APIs instead of editing these. */
/** @brief 驱动内部的命令阶段（N=网络，T=TLS，H=HTTP，S=短信，P=省电）。 */
typedef enum
{
    ML_N_IDLE,
    ML_N_AT,
    ML_N_ECHO,
    ML_N_CMEE,
    ML_N_MODEL,
    ML_N_FW,
    ML_N_IMEI,
    ML_N_SIM,
    ML_N_CFUN,
    ML_N_CSQ,
    ML_N_REG,
    ML_N_PDP,
    ML_N_ACTIVATE,
    ML_N_WAIT,
    ML_T_CLOCK,
    ML_T_LIST,
    ML_T_WRITE,
    ML_T_CERT,
    ML_T_AUTH,
    ML_T_STAMP,
    ML_T_VERIFY,
    ML_T_SNI,
    ML_H_CREATE,
    ML_H_SSL,
    ML_H_CACHE,
    ML_H_ENCODING,
    ML_H_TIMEOUT,
    ML_H_HEADER,
    ML_H_BODY,
    ML_H_REQUEST,
    ML_H_WAIT,
    ML_H_READ,
    ML_H_DELETE,
    ML_S_MODE,
    ML_S_SEND,
    ML_P_SLEEP
} ML307C_Stage;

/** @brief ML307C 驱动对象，包含状态机与全部缓冲区，建议定义为静态变量。 */
typedef struct
{
    ATClient *at;
    ML307C_Callbacks callbacks;
    void *callback_context;
    ML307C_Status status;
    ML307C_State state;
    ML307C_Stage stage;
    bool started, pending, done, value_valid, bad_value, startup_notified;
    bool retry_wait, reboot, network_lost, pdp_notice, sms_channel_uncertain;
    bool tls_cert_present, tls_channel_uncertain;
    bool sleep_requested, sleep_configured, sleeping;
    uint32_t due_ms, wait_ms, health_ms, tls_due_ms, sleep_due_ms;
    ATClient_Result at_result;
    ML307C_HttpConfig http_config;
    const char *path;
    const uint8_t *body;
    size_t body_length;
    char header[128];
    unsigned method;
    int http_id;
    bool response_seen, frame_seen, frame_valid, frame_complete;
    size_t remaining[2], read_size, frame_size, frame_unread, frame_received;
    unsigned read_type;
    ML307C_Result http_result;
    char sms_number[24];
    uint8_t sms_payload[161];
    size_t sms_payload_length;
} ML307C;

/**
 * @brief 初始化驱动对象，并在 ATClient 上注册所需的 URC 与 +MHTTPREAD 定长帧。
 * @param m   传要初始化的 ML307C 变量地址（如 &modem，建议定义为 static），会先被清零。
 * @param at  传已用 ATClient_Init() 初始化的 ATClient 地址（如 &at），其生命周期要长于 m。
 * @param cb  传 ML307C_Callbacks 结构体地址，只填需要的回调即可；不需要任何通知时传 NULL。
 *            内容被复制，可以传局部变量。
 * @param ctx 传任意指针，所有回调的第一个参数都会收到它；不需要时传 NULL。
 * @return true 成功；false 参数为 NULL 或 ATClient 注册表已满。
 * @par 示例
 * @code
 * static ATClient at;
 * static ML307C modem;
 * static const ML307C_Callbacks callbacks = {
 *     .on_state_changed = on_modem_state,
 *     .on_http_complete = on_http_done,
 * };
 * ATClient_Transport transport = ML307C_UartPort_GetTransport();
 * if (ATClient_Init(&at, &transport) && ML307C_Init(&modem, &at, &callbacks, NULL)) {
 *     (void)ML307C_Start(&modem);
 * }
 * @endcode
 */
bool ML307C_Init(ML307C *m, ATClient *at, const ML307C_Callbacks *cb, void *ctx);

/**
 * @brief 启动开机探测与联网流程（AT、ATE0、型号、SIM、CSQ、CEREG、PDP...）。
 * @param m 传已用 ML307C_Init() 初始化的 ML307C 地址（如 &modem）。
 * @return ML307C_RESULT_OK 已启动；
 *         ML307C_RESULT_BUSY 已经启动过；
 *         ML307C_RESULT_INVALID_ARGUMENT m 无效或未初始化。
 * @par 示例
 * @code
 * if (ML307C_Start(&modem) != ML307C_RESULT_OK) {
 *     printf("ML307C start failed\r\n");
 * }
 * @endcode
 */
ML307C_Result ML307C_Start(ML307C *m);

/* Non-blocking: call repeatedly next to ATClient_Process(). All waits are
 * represented by state/deadlines; callbacks execute in the main context. */
/**
 * @brief 驱动主循环处理：推进状态机、处理重试/超时、提交下一条 AT 命令。
 * @param m 传 ML307C 地址（如 &modem）；传 NULL、未启动或睡眠中时直接返回。
 * @return 无
 * @par 示例
 * @code
 * for (;;) {
 *     ATClient_Process(&at);
 *     ML307C_Process(&modem);
 *     ML307C_UartPort_ProcessRaw();
 * }
 * @endcode
 */
void ML307C_Process(ML307C *m);

/* Configure temporary ML307C sleep mode while idle. Only raise DTR after
 * SleepReady returns true; EnterSleep then suppresses all AT polling.
 * Lower DTR before Wake; Wake rechecks network and HTTPS state. */
/**
 * @brief 请求（或取消）空闲睡眠：网络就绪且空闲时，驱动会下发 AT+MLPMCFG 配置睡眠模式。
 * @param m       传 ML307C 地址（如 &modem）；传 NULL 时什么也不做。
 * @param enabled 传 true 表示希望空闲时睡眠；传 false 取消请求。
 * @return 无
 * @par 示例
 * @code
 * ML307C_RequestIdleSleep(&modem, true);
 * if (ML307C_SleepReady(&modem) && ML307C_EnterSleep(&modem)) {
 *     CabinetUI_SetModemSleep(true);   // 拉高 DTR
 * }
 * @endcode
 */
void ML307C_RequestIdleSleep(ML307C *m, bool enabled);

/**
 * @brief 判断现在是否可以拉高 DTR 进入睡眠（已配置睡眠、网络就绪、无进行中的事务）。
 * @param m 传已用 ML307C_Init() 初始化的 ML307C 地址（如 &modem）。
 * @return true 可以进入睡眠；false 不可以或 m 为 NULL。
 * @par 示例
 * @code
 * if (ML307C_SleepReady(&modem)) {
 *     (void)ML307C_EnterSleep(&modem);
 * }
 * @endcode
 */
bool ML307C_SleepReady(const ML307C *m);

/**
 * @brief 标记驱动进入睡眠，之后 ML307C_Process() 不再发送任何 AT 命令。
 * @param m 传已用 ML307C_Init() 初始化的 ML307C 地址（如 &modem）。
 * @return true 已进入睡眠（调用者随后拉高 DTR）；false 条件不满足。
 * @par 示例
 * @code
 * if (ML307C_EnterSleep(&modem)) {
 *     CabinetUI_SetModemSleep(true);   // 拉高 DTR
 * }
 * @endcode
 */
bool ML307C_EnterSleep(ML307C *m);

/**
 * @brief 唤醒驱动：清除睡眠标记并从 AT 探测开始重新确认网络与 HTTPS 状态。
 * @param m 传 ML307C 地址（如 &modem）；传 NULL 或当前未睡眠时什么也不做。
 * @return 无
 * @note  调用前应先拉低 DTR 并等待模组唤醒。
 * @par 示例
 * @code
 * CabinetUI_SetModemSleep(false);      // 拉低 DTR
 * HAL_Delay(100);
 * ML307C_Wake(&modem);
 * @endcode
 */
void ML307C_Wake(ML307C *m);

/**
 * @brief 获取当前网络状态。
 * @param m 传已用 ML307C_Init() 初始化的 ML307C 地址（如 &modem）。
 * @return 当前状态；m 为 NULL 时返回 ML307C_STATE_OFFLINE。
 * @par 示例
 * @code
 * if (ML307C_GetState(&modem) == ML307C_STATE_NETWORK_READY) {
 *     (void)ML307C_HttpGet(&modem, "/ping");
 * }
 * @endcode
 */
ML307C_State ML307C_GetState(const ML307C *m);

/**
 * @brief 获取模组状态快照（只读指针，内容随 Process 更新）。
 * @param m 传已用 ML307C_Init() 初始化的 ML307C 地址（如 &modem）。
 * @return 状态指针；m 为 NULL 时返回 NULL。
 * @par 示例
 * @code
 * const ML307C_Status *st = ML307C_GetStatus(&modem);
 * if (st != NULL && st->rssi_known) {
 *     printf("RSSI %d dBm, IP %s\r\n", st->rssi_dbm, st->ipv4);
 * }
 * @endcode
 */
const ML307C_Status *ML307C_GetStatus(const ML307C *m);

/* Config host remains valid until reconfiguration. path/body remain valid
 * until on_http_complete. Callbacks must return promptly and not block. */
/**
 * @brief 设置 HTTP(S) 服务器地址、读块大小和 HTTPS 根证书。
 * @details host 只能是 "http://域名[:端口]" 或 "https://域名[:端口]"，不能带路径；
 *          https 必须提供 ca_pem / ca_length（≤8192）和 ca_name。重新配置会使 TLS 重新准备。
 * @param m   传 ML307C 地址（如 &modem）；HTTP 请求进行中时不能重新配置。
 * @param cfg 传填好的 ML307C_HttpConfig 地址：host 必填；https 还要填 ca_pem、ca_length、ca_name；
 *            read_chunk_size 填 0 表示默认 256。结构体被复制，但其中的字符串和证书只保存指针，
 *            需长期有效。
 * @return true 配置成功；false 参数非法或 HTTP 忙。
 * @par 示例
 * @code
 * static const ML307C_HttpConfig http = {
 *     .host = "https://api.example.com",
 *     .read_chunk_size = 512U,
 *     .ca_pem = ca_pem,
 *     .ca_length = sizeof(ca_pem) - 1U,
 *     .ca_name = "example_ca_v1.pem",
 * };
 * if (!ML307C_HttpConfigure(&modem, &http)) {
 *     printf("bad HTTP config\r\n");
 * }
 * @endcode
 */
bool ML307C_HttpConfigure(ML307C *m, const ML307C_HttpConfig *cfg);

/**
 * @brief 发起一次异步 HTTP GET 请求。
 * @param m    传已用 ML307C_Init() 初始化的 ML307C 地址（如 &modem）。
 * @param path 传以 '/' 开头的请求路径字符串，如 "/api/v1/status"（≤256 字节、可打印 ASCII，
 *             不能含双引号）；只保存指针，在 on_http_complete 之前不能修改或释放。
 * @return ML307C_RESULT_OK 已开始；
 *         ML307C_RESULT_BUSY 已有 HTTP/短信/命令在进行；
 *         ML307C_RESULT_NOT_READY 网络或 TLS 未就绪、正在睡眠；
 *         ML307C_RESULT_INVALID_ARGUMENT 路径非法。
 * @par 示例
 * @code
 * if (ML307C_HttpGet(&modem, "/api/v1/status") == ML307C_RESULT_OK) {
 *     // 结果通过 on_http_response / on_http_data / on_http_complete 回调返回
 * }
 * @endcode
 */
ML307C_Result ML307C_HttpGet(ML307C *m, const char *path);

/**
 * @brief 发起一次异步 HTTP POST 请求。
 * @param m      传已用 ML307C_Init() 初始化的 ML307C 地址（如 &modem）。
 * @param path   传以 '/' 开头的请求路径字符串，如 "/api/v1/send"；在 on_http_complete
 *               之前保持有效。
 * @param type   传 Content-Type 字符串，如 "application/json"（≤96 字节，内容会被复制）。
 * @param body   传请求正文的首地址（文本可强转为 const uint8_t *）；在 on_http_complete
 *               之前保持有效。
 * @param length 传正文字节数，1..4096（文本一般传 strlen() 或 sizeof(...) - 1）。
 * @return ML307C_RESULT_OK 已开始；
 *         ML307C_RESULT_BUSY 已有 HTTP/短信/命令在进行；
 *         ML307C_RESULT_NOT_READY 网络或 TLS 未就绪、正在睡眠；
 *         ML307C_RESULT_INVALID_ARGUMENT 路径、Content-Type 或正文非法。
 * @par 示例
 * @code
 * static const char json[] = "{\"temp\":25.1}";
 * (void)ML307C_HttpPost(&modem, "/api/v1/report", "application/json",
 *                       (const uint8_t *)json, sizeof(json) - 1U);
 * @endcode
 */
ML307C_Result ML307C_HttpPost(ML307C *m, const char *path, const char *type, const uint8_t *body,
                              size_t length);

/* Non-blocking, one SMS at a time. Printable ASCII text, 1..160 bytes.
 * No automatic retry: a timeout may occur after the network accepted it. */
/**
 * @brief 发送一条文本短信（非阻塞，同一时间只能有一条）。
 * @param m      传已用 ML307C_Init() 初始化的 ML307C 地址（如 &modem）。
 * @param number 传对方手机号字符串，如 "13800138000" 或 "+8613800138000"（1..20 位数字，
 *               内容被复制）。
 * @param text   传短信正文，只能是可打印 ASCII，1..160 字节（内容被复制，不支持中文）。
 * @return ML307C_RESULT_OK 已开始；
 *         ML307C_RESULT_BUSY 有短信/HTTP/命令在进行；
 *         ML307C_RESULT_NOT_READY 网络或 SIM 未就绪、正在睡眠；
 *         ML307C_RESULT_INVALID_ARGUMENT 号码或文本非法。
 * @note  超时不会自动重发，因为网络可能已经接收。结果通过 on_sms_complete 返回。
 * @par 示例
 * @code
 * (void)ML307C_SendSms(&modem, "+8613800000000", "Humidity alarm: 85%");
 * @endcode
 */
ML307C_Result ML307C_SendSms(ML307C *m, const char *number, const char *text);
#ifdef __cplusplus
}
#endif
#endif
