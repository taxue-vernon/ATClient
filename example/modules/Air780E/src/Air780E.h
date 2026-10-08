/**
 * @file    Air780E.h
 * @brief   合宙 Air780E 4G 模组驱动：开机探测、驻网检测与 MQTT（连接/订阅/发布/断开/自动重连）。
 * @details 基于 ATClient 的非阻塞状态机。启动流程依次发送
 *          AT、ATE0、AT+CMEE=2、ATI、AT+CGMR、AT+CGSN、AT+CPIN?、AT+CEREG?、AT+CGATT?；
 *          启动完成后可配置并连接 MQTT。主循环需同时调用 ATClient_Process() 与 Air780E_Process()。
 */

#ifndef AIR780E_H
#define AIR780E_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ATClient.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AIR780E_MODEL_CAPACITY    32U
#define AIR780E_FIRMWARE_CAPACITY 64U
#define AIR780E_IMEI_CAPACITY     16U
#define AIR780E_MQTT_FIELD_MAX    256U

/**
 * @brief MQTT 连接参数。字符串只保存指针，需在句柄生命周期内（或下次配置前）保持有效。
 * @details host/client_id 不能为空；username/password 可为空串；port 与 keepalive_s 取 1..65535；
 *          auto_reconnect 为 true 时，连接失败或被动断开后按 1/2/4/8/16/30 秒退避重连。
 */
typedef struct
{
    const char *host;
    uint32_t port;
    const char *client_id;
    const char *username;
    const char *password;
    bool clean_session;
    uint32_t keepalive_s;
    bool auto_reconnect;
} Air780E_MqttConfig;

/** @brief Air780E 对外可见的状态，变化时触发 on_state_changed。 */
typedef enum
{
    AIR780E_STATE_OFFLINE = 0,
    AIR780E_STATE_PROBING,
    AIR780E_STATE_SIM_WAIT,
    AIR780E_STATE_NETWORK_WAIT,
    AIR780E_STATE_MQTT_TCP_CONNECTING,
    AIR780E_STATE_MQTT_AUTHENTICATING,
    AIR780E_STATE_MQTT_ONLINE,
    AIR780E_STATE_RECONNECT_WAIT,
    AIR780E_STATE_ERROR
} Air780E_State;

/** @brief Air780E 接口与回调使用的结果码。 */
typedef enum
{
    AIR780E_RESULT_OK = 0,
    AIR780E_RESULT_BUSY,
    AIR780E_RESULT_INVALID_ARGUMENT,
    AIR780E_RESULT_AT_ERROR,
    AIR780E_RESULT_TIMEOUT,
    AIR780E_RESULT_PARSE_ERROR,
    AIR780E_RESULT_NOT_READY
} Air780E_Result;

/** @brief MQTT 操作类型，用于 on_mqtt_operation_complete 区分是哪种操作完成。 */
typedef enum
{
    AIR780E_MQTT_OPERATION_NONE = 0,
    AIR780E_MQTT_OPERATION_CONNECT,
    AIR780E_MQTT_OPERATION_SUBSCRIBE,
    AIR780E_MQTT_OPERATION_PUBLISH,
    AIR780E_MQTT_OPERATION_DISCONNECT,
    AIR780E_MQTT_OPERATION_CACHE_GET
} Air780E_MqttOperation;

/** @brief MQTT 操作内部的 AT 命令步骤。 */
typedef enum
{
    AIR780E_MQTT_STEP_IDLE = 0,
    AIR780E_MQTT_STEP_MCONFIG,
    AIR780E_MQTT_STEP_MSGSET,
    AIR780E_MQTT_STEP_MIPSTART,
    AIR780E_MQTT_STEP_MCONNECT,
    AIR780E_MQTT_STEP_MSUB,
    AIR780E_MQTT_STEP_MPUBEX,
    AIR780E_MQTT_STEP_MDISCONNECT,
    AIR780E_MQTT_STEP_MIPCLOSE,
    AIR780E_MQTT_STEP_MSGGET
} Air780E_MqttStep;

/** @brief 模组状态快照（型号、固件、IMEI、SIM、信号、注册、MQTT 统计等），只读。 */
typedef struct
{
    bool responsive;
    char model[AIR780E_MODEL_CAPACITY];
    char firmware[AIR780E_FIRMWARE_CAPACITY];
    char imei[AIR780E_IMEI_CAPACITY];
    bool sim_ready;
    bool rssi_known;
    uint8_t rssi;
    uint8_t ber;
    int16_t rssi_dbm;
    uint8_t registration;
    bool packet_attached;
    bool mqtt_online;
    Air780E_MqttOperation last_mqtt_operation;
    Air780E_Result last_result;
    uint32_t retry_count;
    uint32_t parse_error_count;
    uint32_t mqtt_message_count;
    uint32_t mqtt_cache_overwrite_risk;
    uint32_t mqtt_reconnect_count;
} Air780E_Status;

/**
 * @brief 异步事件回调集合，任一成员可为 NULL；第一个参数都是 Air780E_Init() 的 callback_context。
 * @details - on_state_changed：状态变化；
 *          - on_startup_complete：启动探测结束（成功或失败）；
 *          - on_mqtt_operation_complete：连接/订阅/发布/断开/缓存读取结束；
 *          - on_mqtt_message_begin / data / end：收到一条订阅消息（topic、分片数据、结束）。
 */
typedef struct
{
    void (*on_state_changed)(void *context, Air780E_State state);
    void (*on_startup_complete)(void *context, Air780E_Result result);
    void (*on_mqtt_operation_complete)(void *context, Air780E_MqttOperation operation,
                                       Air780E_Result result);
    void (*on_mqtt_message_begin)(void *context, const char *topic, size_t payload_length);
    void (*on_mqtt_message_data)(void *context, const uint8_t *data, size_t length);
    void (*on_mqtt_message_end)(void *context, Air780E_Result result);
} Air780E_Callbacks;

/** @brief 启动探测阶段，与发送的 AT 命令一一对应。 */
typedef enum
{
    AIR780E_STAGE_IDLE = 0,
    AIR780E_STAGE_AT,
    AIR780E_STAGE_ATE0,
    AIR780E_STAGE_CMEE,
    AIR780E_STAGE_ATI,
    AIR780E_STAGE_CGMR,
    AIR780E_STAGE_CGSN,
    AIR780E_STAGE_CPIN,
    AIR780E_STAGE_CEREG,
    AIR780E_STAGE_CGATT,
    AIR780E_STAGE_COMPLETE
} Air780E_Stage;

/** @brief Air780E 驱动对象，建议定义为静态变量。 */
typedef struct
{
    ATClient *at;
    Air780E_Callbacks callbacks;
    void *callback_context;
    Air780E_State state;
    Air780E_Status status;
    Air780E_Stage stage;
    ATClient_Request request;
    bool started;
    bool command_pending;
    bool command_active;
    bool stage_value_valid;
    bool retry_waiting;
    uint8_t stage_retries;
    uint32_t retry_due_ms;
    Air780E_MqttConfig mqtt_config;
    bool mqtt_configured;
    Air780E_MqttOperation mqtt_operation;
    Air780E_MqttStep mqtt_step;
    const char *mqtt_topic;
    uint8_t mqtt_qos;
    const uint8_t *mqtt_payload;
    size_t mqtt_payload_length;
    bool mqtt_retain;
    ATClient_Request mqtt_get_request;
    bool mqtt_get_queued;
    uint8_t mqtt_cache_notices;
    char mqtt_rx_topic[AIR780E_MQTT_FIELD_MAX + 1U];
    bool mqtt_reconnect_waiting;
    bool mqtt_user_disconnect;
    uint8_t mqtt_reconnect_index;
    uint32_t mqtt_reconnect_due_ms;
} Air780E;

/**
 * @brief 初始化驱动对象，并在 ATClient 上注册 +MSUB 定长帧、CLOSED 与 +MQTTSTATU 等 URC。
 * @param modem            传要初始化的 Air780E 变量地址（如 &modem，建议定义为 static），
 *                         会先被清零。
 * @param at               传已用 ATClient_Init() 初始化的 ATClient 地址（如 &at）。
 * @param callbacks        传 Air780E_Callbacks 结构体地址，只填需要的回调；不需要通知时传
 *                         NULL（内容被复制）。
 * @param callback_context 传任意指针，所有回调的第一个参数都会收到它；不需要时传 NULL。
 * @return true 成功；false 参数为 NULL 或 ATClient 注册表已满。
 * @par 示例
 * @code
 * static ATClient at;
 * static Air780E modem;
 * static const Air780E_Callbacks callbacks = {
 *     .on_startup_complete = on_startup,
 *     .on_mqtt_message_data = on_message_data,
 * };
 * ATClient_Transport transport = Air780E_UartPort_GetTransport();
 * if (ATClient_Init(&at, &transport) && Air780E_Init(&modem, &at, &callbacks, NULL)) {
 *     (void)Air780E_Start(&modem);
 * }
 * @endcode
 */
bool Air780E_Init(Air780E *modem, ATClient *at, const Air780E_Callbacks *callbacks,
                  void *callback_context);

/**
 * @brief 开始启动探测流程（AT -> ... -> AT+CGATT?），结果通过 on_startup_complete 返回。
 * @param modem 传已用 Air780E_Init() 初始化的 Air780E 地址（如 &modem）。
 * @return AIR780E_RESULT_OK 已开始；
 *         AIR780E_RESULT_BUSY 正在启动或有命令未完成；
 *         AIR780E_RESULT_INVALID_ARGUMENT 参数无效。
 * @par 示例
 * @code
 * if (Air780E_Start(&modem) != AIR780E_RESULT_OK) {
 *     printf("Air780E busy\r\n");
 * }
 * @endcode
 */
Air780E_Result Air780E_Start(Air780E *modem);

/* Configuration strings are borrowed and must remain valid for the lifetime
 * of the Air780E handle or until the next successful configuration call. */
/**
 * @brief 保存 MQTT 连接参数（只校验，不发送命令）。
 * @param modem  传 Air780E 地址（如 &modem）；启动探测或命令执行期间不能配置。
 * @param config 传填好的 Air780E_MqttConfig 地址（各字段约束见该结构体说明）；
 *               结构体被复制，但其中的字符串只保存指针，需长期有效。
 * @return AIR780E_RESULT_OK 成功；BUSY 忙；INVALID_ARGUMENT 参数非法或命令会超长。
 * @par 示例
 * @code
 * static const Air780E_MqttConfig mqtt = {
 *     .host = "broker.example.com",
 *     .port = 1883U,
 *     .client_id = "cube-001",
 *     .username = "",
 *     .password = "",
 *     .clean_session = true,
 *     .keepalive_s = 60U,
 *     .auto_reconnect = true,
 * };
 * (void)Air780E_MqttConfigure(&modem, &mqtt);
 * @endcode
 */
Air780E_Result Air780E_MqttConfigure(Air780E *modem, const Air780E_MqttConfig *config);

/**
 * @brief 异步连接 MQTT 服务器（MCONFIG -> MQTTMSGSET -> MIPSTART -> MCONNECT）。
 * @param modem 传已用 Air780E_Init() 初始化的 Air780E 地址（如 &modem）。
 * @return AIR780E_RESULT_OK 已开始；
 *         AIR780E_RESULT_NOT_READY 未配置或未附着分组域；
 *         AIR780E_RESULT_BUSY 有其他操作在进行；
 *         AIR780E_RESULT_INVALID_ARGUMENT 参数无效。
 * @par 示例
 * @code
 * if (Air780E_MqttConnect(&modem) == AIR780E_RESULT_OK) {
 *     // 结果在 on_mqtt_operation_complete(ctx, AIR780E_MQTT_OPERATION_CONNECT, result) 中返回
 * }
 * @endcode
 */
Air780E_Result Air780E_MqttConnect(Air780E *modem);

/* Topic storage is borrowed until the completion callback for this operation. */
/**
 * @brief 异步订阅一个主题。
 * @param modem 传 Air780E 地址（如 &modem），必须已经 MQTT 在线。
 * @param topic 传主题字符串，如 "cube/cmd"；不能含双引号或控制字符，在完成回调前保持有效。
 * @param qos   传服务质量 0、1 或 2。
 * @return AIR780E_RESULT_OK 已开始；NOT_READY 未在线；BUSY 忙；INVALID_ARGUMENT 参数非法。
 * @par 示例
 * @code
 * (void)Air780E_MqttSubscribe(&modem, "cube/cmd", 1U);
 * @endcode
 */
Air780E_Result Air780E_MqttSubscribe(Air780E *modem, const char *topic, uint8_t qos);

/* Topic and payload are borrowed until the publish completion callback. */
/**
 * @brief 异步发布一条消息（AT+MPUBEX，提示符后发送二进制负载）。
 * @param modem   传 Air780E 地址（如 &modem），必须已经 MQTT 在线。
 * @param topic   传主题字符串，如 "cube/status"；在完成回调前保持有效。
 * @param payload 传消息内容首地址（文本可强转为 const uint8_t *）；在完成回调前保持有效。
 * @param length  传消息字节数，1..ATCLIENT_FRAME_PAYLOAD_MAX（文本一般传 strlen()）。
 * @param qos     传 0、1 或 2（1 等待 PUBACK，2 等待 PUBREC/PUBCOMP）。
 * @param retain  传 true 让服务器保留这条消息，一般传 false。
 * @return AIR780E_RESULT_OK 已开始；NOT_READY 未在线；BUSY 忙；INVALID_ARGUMENT 参数非法。
 * @par 示例
 * @code
 * static const char msg[] = "{\"rh\":65}";
 * (void)Air780E_MqttPublish(&modem, "cube/status", (const uint8_t *)msg,
 *                           sizeof(msg) - 1U, 1U, false);
 * @endcode
 */
Air780E_Result Air780E_MqttPublish(Air780E *modem, const char *topic, const uint8_t *payload,
                                   size_t length, uint8_t qos, bool retain);

/**
 * @brief 主动断开 MQTT（MDISCONNECT -> MIPCLOSE），主动断开后不会自动重连。
 * @param modem 传 Air780E 地址（如 &modem），必须已经 MQTT 在线。
 * @return AIR780E_RESULT_OK 已开始；NOT_READY 未在线；BUSY 忙；INVALID_ARGUMENT 参数无效。
 * @par 示例
 * @code
 * (void)Air780E_MqttDisconnect(&modem);
 * @endcode
 */
Air780E_Result Air780E_MqttDisconnect(Air780E *modem);

/**
 * @brief 驱动主循环处理：读取缓存消息、到期重连、重试等待，并提交下一条命令。
 * @param modem 传 Air780E 地址（如 &modem）；传 NULL 时什么也不做。
 * @return 无
 * @par 示例
 * @code
 * for (;;) {
 *     ATClient_Process(&at);
 *     Air780E_Process(&modem);
 *     Air780E_UartPort_ProcessRaw();
 * }
 * @endcode
 */
void Air780E_Process(Air780E *modem);

/**
 * @brief 获取当前状态。
 * @param modem 传 Air780E 地址（如 &modem）；传 NULL 时返回 ERROR。
 * @return 当前状态；modem 为 NULL 时返回 AIR780E_STATE_ERROR。
 * @par 示例
 * @code
 * if (Air780E_GetState(&modem) == AIR780E_STATE_MQTT_ONLINE) {
 *     publish_report();
 * }
 * @endcode
 */
Air780E_State Air780E_GetState(const Air780E *modem);

/**
 * @brief 获取模组状态快照。
 * @param modem 传 Air780E 地址（如 &modem）；传 NULL 时返回 NULL。
 * @return 状态指针；modem 为 NULL 时返回 NULL。
 * @par 示例
 * @code
 * const Air780E_Status *st = Air780E_GetStatus(&modem);
 * if (st != NULL) {
 *     printf("IMEI %s, CSQ %u\r\n", st->imei, st->rssi);
 * }
 * @endcode
 */
const Air780E_Status *Air780E_GetStatus(const Air780E *modem);

#ifdef __cplusplus
}
#endif

#endif
