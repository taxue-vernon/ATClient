/**
 * @file  Air780E.c
 * @brief 合宙 Air780E 驱动实现。公开接口的形参说明与示例见 Air780E.h。
 */

#include "Air780E.h"

#include <stdio.h>
#include <string.h>

#define AIR780E_COMMAND_TIMEOUT_MS 5000U
#define AIR780E_RETRY_DELAY_MS     1000U
#define AIR780E_MAX_STAGE_RETRIES  3U

static bool Air780E_BuildCommand(void *context, char *destination, size_t capacity, size_t *length);
static void Air780E_HandleLine(void *context, const char *line, size_t length);
static void Air780E_CommandComplete(void *context, ATClient_Result result);
static const char *Air780E_CommandForStage(Air780E_Stage stage);
static bool Air780E_BuildMqttCommand(Air780E *modem, char *destination, size_t capacity,
                                     size_t *length);
static void Air780E_PrepareMqttRequest(Air780E *modem);
static void Air780E_MqttCommandComplete(Air780E *modem, ATClient_Result result);
static void Air780E_FinishMqttOperation(Air780E *modem, Air780E_Result result);
static void Air780E_AdvanceStage(Air780E *modem);
static void Air780E_SetState(Air780E *modem, Air780E_State state);
static void Air780E_FinishStartup(Air780E *modem, Air780E_Result result);
static void Air780E_ScheduleRetry(Air780E *modem, Air780E_Result result);
static Air780E_Result Air780E_MapAtResult(ATClient_Result result);
static bool Air780E_CopyField(char *destination, size_t capacity, const char *source,
                              size_t length);
static bool Air780E_ParseUnsigned(const char **cursor, const char *end, unsigned int *value);
static bool Air780E_ParseCereg(Air780E *modem, const char *line, size_t length);
static bool Air780E_ParseCgatt(Air780E *modem, const char *line, size_t length);
static bool Air780E_ParseCsq(Air780E *modem, const char *line, size_t length);
static bool Air780E_MqttFieldValid(const char *value, bool allow_empty);
static void Air780E_SubmitCachedGet(Air780E *modem);
static bool Air780E_BuildMqttGet(void *context, char *destination, size_t capacity, size_t *length);
static void Air780E_MqttGetComplete(void *context, ATClient_Result result);
static void Air780E_CacheUrc(void *context, const char *line, size_t length);
static ATClient_FrameProbeResult Air780E_MqttFrameProbe(void *context, const uint8_t *header,
                                                        size_t header_length,
                                                        size_t *payload_length);
static void Air780E_MqttFrameBegin(void *context, const uint8_t *header, size_t header_length,
                                   size_t payload_length);
static void Air780E_MqttFrameData(void *context, const uint8_t *data, size_t length);
static void Air780E_MqttFrameEnd(void *context, ATClient_Result result);
static void Air780E_ClosedUrc(void *context, const char *line, size_t length);
static void Air780E_MqttStatusUrc(void *context, const char *line, size_t length);
static void Air780E_ScheduleMqttReconnect(Air780E *modem);

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
                  void *callback_context)
{
    ATClient_LengthFrameRegistration frame_registration;

    if (modem == NULL || at == NULL) {
        return false;
    }

    memset(modem, 0, sizeof(*modem));
    modem->at = at;
    if (callbacks != NULL) {
        modem->callbacks = *callbacks;
    }
    modem->callback_context = callback_context;
    modem->state = AIR780E_STATE_OFFLINE;
    modem->status.rssi = 99U;
    modem->status.ber = 99U;
    modem->status.last_result = AIR780E_RESULT_NOT_READY;
    memset(&frame_registration, 0, sizeof(frame_registration));
    frame_registration.prefix = "+MSUB:";
    frame_registration.probe = Air780E_MqttFrameProbe;
    frame_registration.on_begin = Air780E_MqttFrameBegin;
    frame_registration.on_data = Air780E_MqttFrameData;
    frame_registration.on_end = Air780E_MqttFrameEnd;
    frame_registration.context = modem;
    return ATClient_RegisterLengthFrame(at, &frame_registration) == ATCLIENT_RESULT_OK &&
           ATClient_RegisterUrc(at, "+MSUB:", Air780E_CacheUrc, modem) == ATCLIENT_RESULT_OK &&
           ATClient_RegisterUrc(at, "CLOSED", Air780E_ClosedUrc, modem) == ATCLIENT_RESULT_OK &&
           ATClient_RegisterUrc(at, "+MQTTSTATU:", Air780E_MqttStatusUrc, modem) ==
               ATCLIENT_RESULT_OK;
}

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
Air780E_Result Air780E_Start(Air780E *modem)
{
    if (modem == NULL || modem->at == NULL) {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    if (modem->started || modem->command_active || modem->command_pending) {
        return AIR780E_RESULT_BUSY;
    }

    memset(&modem->status, 0, sizeof(modem->status));
    modem->status.rssi = 99U;
    modem->status.ber = 99U;
    modem->status.last_result = AIR780E_RESULT_NOT_READY;
    modem->stage = AIR780E_STAGE_AT;
    modem->started = true;
    modem->command_pending = true;
    modem->stage_retries = 0U;
    modem->retry_waiting = false;
    Air780E_SetState(modem, AIR780E_STATE_PROBING);
    return AIR780E_RESULT_OK;
}

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
Air780E_Result Air780E_MqttConfigure(Air780E *modem, const Air780E_MqttConfig *config)
{
    size_t mconfig_length;
    size_t mipstart_length;

    if (modem == NULL || config == NULL || config->port == 0U || config->port > 65535U ||
        config->keepalive_s == 0U || config->keepalive_s > 65535U ||
        !Air780E_MqttFieldValid(config->host, false) ||
        !Air780E_MqttFieldValid(config->client_id, false) ||
        !Air780E_MqttFieldValid(config->username, true) ||
        !Air780E_MqttFieldValid(config->password, true)) {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    if (modem->started || modem->command_active || modem->command_pending) {
        return AIR780E_RESULT_BUSY;
    }

    mconfig_length = strlen("AT+MCONFIG=\"\",\"\",\"\"") + strlen(config->client_id) +
                     strlen(config->username) + strlen(config->password);
    mipstart_length = strlen("AT+MIPSTART=\"\",65535") + strlen(config->host);
    if (mconfig_length > ATCLIENT_COMMAND_CAPACITY || mipstart_length > ATCLIENT_COMMAND_CAPACITY) {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }

    modem->mqtt_config = *config;
    modem->mqtt_configured = true;
    return AIR780E_RESULT_OK;
}

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
Air780E_Result Air780E_MqttConnect(Air780E *modem)
{
    if (modem == NULL) {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    if (!modem->mqtt_configured || !modem->status.packet_attached) {
        return AIR780E_RESULT_NOT_READY;
    }
    if (modem->started || modem->mqtt_operation != AIR780E_MQTT_OPERATION_NONE ||
        modem->command_active || modem->command_pending) {
        return AIR780E_RESULT_BUSY;
    }

    modem->mqtt_operation = AIR780E_MQTT_OPERATION_CONNECT;
    modem->mqtt_step = AIR780E_MQTT_STEP_MCONFIG;
    modem->command_pending = true;
    modem->mqtt_user_disconnect = false;
    modem->mqtt_reconnect_waiting = false;
    modem->mqtt_reconnect_index = 0U;
    Air780E_SetState(modem, AIR780E_STATE_MQTT_TCP_CONNECTING);
    return AIR780E_RESULT_OK;
}

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
Air780E_Result Air780E_MqttSubscribe(Air780E *modem, const char *topic, uint8_t qos)
{
    if (modem == NULL || qos > 2U || !Air780E_MqttFieldValid(topic, false)) {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    if (!modem->status.mqtt_online) {
        return AIR780E_RESULT_NOT_READY;
    }
    if (modem->started || modem->mqtt_operation != AIR780E_MQTT_OPERATION_NONE ||
        modem->command_active || modem->command_pending) {
        return AIR780E_RESULT_BUSY;
    }

    modem->mqtt_topic = topic;
    modem->mqtt_qos = qos;
    modem->mqtt_operation = AIR780E_MQTT_OPERATION_SUBSCRIBE;
    modem->mqtt_step = AIR780E_MQTT_STEP_MSUB;
    modem->command_pending = true;
    return AIR780E_RESULT_OK;
}

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
                                   size_t length, uint8_t qos, bool retain)
{
    if (modem == NULL || payload == NULL || length == 0U || length > ATCLIENT_FRAME_PAYLOAD_MAX ||
        qos > 2U || !Air780E_MqttFieldValid(topic, false)) {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    if (!modem->status.mqtt_online) {
        return AIR780E_RESULT_NOT_READY;
    }
    if (modem->started || modem->mqtt_operation != AIR780E_MQTT_OPERATION_NONE ||
        modem->command_active || modem->command_pending) {
        return AIR780E_RESULT_BUSY;
    }

    modem->mqtt_topic = topic;
    modem->mqtt_qos = qos;
    modem->mqtt_payload = payload;
    modem->mqtt_payload_length = length;
    modem->mqtt_retain = retain;
    modem->mqtt_operation = AIR780E_MQTT_OPERATION_PUBLISH;
    modem->mqtt_step = AIR780E_MQTT_STEP_MPUBEX;
    modem->command_pending = true;
    return AIR780E_RESULT_OK;
}

/**
 * @brief 主动断开 MQTT（MDISCONNECT -> MIPCLOSE），主动断开后不会自动重连。
 * @param modem 传 Air780E 地址（如 &modem），必须已经 MQTT 在线。
 * @return AIR780E_RESULT_OK 已开始；NOT_READY 未在线；BUSY 忙；INVALID_ARGUMENT 参数无效。
 * @par 示例
 * @code
 * (void)Air780E_MqttDisconnect(&modem);
 * @endcode
 */
Air780E_Result Air780E_MqttDisconnect(Air780E *modem)
{
    if (modem == NULL) {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    if (!modem->status.mqtt_online) {
        return AIR780E_RESULT_NOT_READY;
    }
    if (modem->started || modem->mqtt_operation != AIR780E_MQTT_OPERATION_NONE ||
        modem->command_active || modem->command_pending) {
        return AIR780E_RESULT_BUSY;
    }

    modem->mqtt_user_disconnect = true;
    modem->mqtt_reconnect_waiting = false;
    modem->mqtt_operation = AIR780E_MQTT_OPERATION_DISCONNECT;
    modem->mqtt_step = AIR780E_MQTT_STEP_MDISCONNECT;
    modem->command_pending = true;
    return AIR780E_RESULT_OK;
}

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
void Air780E_Process(Air780E *modem)
{
    ATClient_Result submit_result;

    if (modem == NULL) {
        return;
    }
    Air780E_SubmitCachedGet(modem);
    /* Non-blocking reconnect: compare the injected tick and enqueue work only;
     * never delay or wait here for the modem or network. */
    if (modem->mqtt_reconnect_waiting && !modem->started &&
        modem->mqtt_operation == AIR780E_MQTT_OPERATION_NONE &&
        (int32_t)(ATClient_NowMs(modem->at) - modem->mqtt_reconnect_due_ms) >= 0) {
        modem->mqtt_reconnect_waiting = false;
        ++modem->status.mqtt_reconnect_count;
        modem->mqtt_operation = AIR780E_MQTT_OPERATION_CONNECT;
        modem->mqtt_step = AIR780E_MQTT_STEP_MCONFIG;
        modem->command_pending = true;
        Air780E_SetState(modem, AIR780E_STATE_MQTT_TCP_CONNECTING);
    }
    if (modem->command_active) {
        return;
    }

    if (!modem->started && modem->mqtt_operation == AIR780E_MQTT_OPERATION_NONE) {
        return;
    }

    if (modem->retry_waiting) {
        uint32_t now_ms = ATClient_NowMs(modem->at);

        if ((int32_t)(now_ms - modem->retry_due_ms) < 0) {
            return;
        }
        modem->retry_waiting = false;
        modem->command_pending = true;
    }

    if (!modem->command_pending) {
        return;
    }

    memset(&modem->request, 0, sizeof(modem->request));
    modem->request.build_command = Air780E_BuildCommand;
    modem->request.command_context = modem;
    modem->request.timeout_ms = AIR780E_COMMAND_TIMEOUT_MS;
    modem->request.priority = ATCLIENT_PRIORITY_NORMAL;
    modem->request.on_line = Air780E_HandleLine;
    modem->request.on_complete = Air780E_CommandComplete;
    modem->request.callback_context = modem;
    modem->stage_value_valid = false;
    if (!modem->started) {
        Air780E_PrepareMqttRequest(modem);
    }

    submit_result = ATClient_Submit(modem->at, &modem->request);
    /* Non-blocking: submission only queues the transaction. UART completion
     * and protocol results arrive later through ATClient_Process(). */
    if (submit_result == ATCLIENT_RESULT_OK) {
        modem->command_pending = false;
        modem->command_active = true;
    } else if (submit_result != ATCLIENT_RESULT_QUEUE_FULL) {
        Air780E_FinishStartup(modem, Air780E_MapAtResult(submit_result));
    }
}

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
Air780E_State Air780E_GetState(const Air780E *modem)
{
    return modem == NULL ? AIR780E_STATE_ERROR : modem->state;
}

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
const Air780E_Status *Air780E_GetStatus(const Air780E *modem)
{
    return modem == NULL ? NULL : &modem->status;
}

/**
 * @brief ATClient 命令构造回调：启动阶段取固定命令，MQTT 阶段交给 Air780E_BuildMqttCommand()。
 * @param context     由 ATClient 传入：注册/提交时给的 Air780E 对象指针。
 * @param destination 由 ATClient 传入：命令写入的缓冲区。
 * @param capacity    由 ATClient 传入：destination 的容量。
 * @param length      输出：写入命令长度。
 * @return true 成功；false 无命令或缓冲区不足。
 * @par 示例
 * @code
 * modem->request.build_command = Air780E_BuildCommand;
 * modem->request.command_context = modem;
 * @endcode
 */
static bool Air780E_BuildCommand(void *context, char *destination, size_t capacity, size_t *length)
{
    Air780E *modem = (Air780E *)context;
    const char *command;
    size_t command_length;

    if (!modem->started) {
        return Air780E_BuildMqttCommand(modem, destination, capacity, length);
    }

    command = Air780E_CommandForStage(modem->stage);

    if (command == NULL) {
        return false;
    }
    command_length = strlen(command);
    if (command_length > capacity) {
        return false;
    }
    memcpy(destination, command, command_length);
    *length = command_length;
    return true;
}

/**
 * @brief 响应行回调：解析 +CSQ、+CPIN、+CEREG、+CGATT，以及 ATI/CGMR/CGSN 的文本结果。
 * @param context 由 ATClient 传入：注册/提交时给的 Air780E 对象指针。
 * @param line    由 ATClient 传入：以 '\0' 结尾的响应行，如 "+CSQ: 20,99"。
 * @param length  由 ATClient 传入：line 的字节数。
 * @return 无
 * @par 示例
 * @code
 * modem->request.on_line = Air780E_HandleLine;
 * @endcode
 */
static void Air780E_HandleLine(void *context, const char *line, size_t length)
{
    Air780E *modem = (Air780E *)context;

    if (length >= 6U && memcmp(line, "+CSQ:", 5U) == 0) {
        (void)Air780E_ParseCsq(modem, line, length);
        return;
    }
    if (length >= 7U && memcmp(line, "+CPIN:", 6U) == 0) {
        static const char ready[] = "+CPIN: READY";

        modem->status.sim_ready =
            length == sizeof(ready) - 1U && memcmp(line, ready, sizeof(ready) - 1U) == 0;
        modem->stage_value_valid = true;
        return;
    }
    if (length >= 8U && memcmp(line, "+CEREG:", 7U) == 0) {
        modem->stage_value_valid = Air780E_ParseCereg(modem, line, length);
        return;
    }
    if (length >= 8U && memcmp(line, "+CGATT:", 7U) == 0) {
        modem->stage_value_valid = Air780E_ParseCgatt(modem, line, length);
        return;
    }

    if (modem->stage == AIR780E_STAGE_ATI) {
        modem->stage_value_valid =
            Air780E_CopyField(modem->status.model, sizeof(modem->status.model), line, length);
    } else if (modem->stage == AIR780E_STAGE_CGMR) {
        modem->stage_value_valid =
            Air780E_CopyField(modem->status.firmware, sizeof(modem->status.firmware), line, length);
    } else if (modem->stage == AIR780E_STAGE_CGSN) {
        size_t index;

        modem->stage_value_valid = length == 15U;
        for (index = 0U; index < length && modem->stage_value_valid; ++index) {
            modem->stage_value_valid = line[index] >= '0' && line[index] <= '9';
        }
        if (modem->stage_value_valid) {
            (void)Air780E_CopyField(modem->status.imei, sizeof(modem->status.imei), line, length);
        }
    }
}

/**
 * @brief 命令完成回调：启动阶段校验结果并推进/重试，MQTT 阶段转交 Air780E_MqttCommandComplete()。
 * @param context 由 ATClient 传入：注册/提交时给的 Air780E 对象指针。
 * @param result  由 ATClient 传入：这条命令的完成结果。
 * @return 无
 * @par 示例
 * @code
 * modem->request.on_complete = Air780E_CommandComplete;
 * @endcode
 */
static void Air780E_CommandComplete(void *context, ATClient_Result result)
{
    Air780E *modem = (Air780E *)context;

    modem->command_active = false;
    if (!modem->started) {
        Air780E_MqttCommandComplete(modem, result);
        return;
    }
    if (result != ATCLIENT_RESULT_OK) {
        Air780E_ScheduleRetry(modem, Air780E_MapAtResult(result));
        return;
    }

    if (modem->stage == AIR780E_STAGE_AT) {
        modem->status.responsive = true;
    }

    if ((modem->stage == AIR780E_STAGE_ATI || modem->stage == AIR780E_STAGE_CGMR ||
         modem->stage == AIR780E_STAGE_CGSN || modem->stage == AIR780E_STAGE_CPIN ||
         modem->stage == AIR780E_STAGE_CEREG || modem->stage == AIR780E_STAGE_CGATT) &&
        !modem->stage_value_valid) {
        ++modem->status.parse_error_count;
        Air780E_FinishStartup(modem, AIR780E_RESULT_PARSE_ERROR);
        return;
    }

    if (modem->stage == AIR780E_STAGE_CPIN && !modem->status.sim_ready) {
        Air780E_SetState(modem, AIR780E_STATE_SIM_WAIT);
        Air780E_ScheduleRetry(modem, AIR780E_RESULT_NOT_READY);
        return;
    }
    if (modem->stage == AIR780E_STAGE_CEREG && modem->status.registration != 1U &&
        modem->status.registration != 5U) {
        Air780E_SetState(modem, AIR780E_STATE_NETWORK_WAIT);
        Air780E_ScheduleRetry(modem, AIR780E_RESULT_NOT_READY);
        return;
    }
    if (modem->stage == AIR780E_STAGE_CGATT && !modem->status.packet_attached) {
        Air780E_SetState(modem, AIR780E_STATE_NETWORK_WAIT);
        Air780E_ScheduleRetry(modem, AIR780E_RESULT_NOT_READY);
        return;
    }

    modem->stage_retries = 0U;
    Air780E_AdvanceStage(modem);
}

/**
 * @brief 按当前 MQTT 步骤生成 AT+MCONFIG / MIPSTART / MCONNECT / MSUB / MPUBEX 等命令。
 * @param modem       传当前 Air780E 对象指针（内部调用时就是公开接口收到的 modem）。
 * @param destination 传 BuildCommand 收到的输出缓冲区。
 * @param capacity    传 destination 的容量。
 * @param length      输出：传 BuildCommand 收到的 length 指针，用于写入命令长度。
 * @return true 成功；false 无对应命令或缓冲区不足。
 * @par 示例
 * @code
 * if (!modem->started) {
 *     return Air780E_BuildMqttCommand(modem, destination, capacity, length);
 * }
 * @endcode
 */
static bool Air780E_BuildMqttCommand(Air780E *modem, char *destination, size_t capacity,
                                     size_t *length)
{
    int written = -1;

    switch (modem->mqtt_step) {
    case AIR780E_MQTT_STEP_MCONFIG:
        written = snprintf(destination, capacity, "AT+MCONFIG=\"%s\",\"%s\",\"%s\"",
                           modem->mqtt_config.client_id, modem->mqtt_config.username,
                           modem->mqtt_config.password);
        break;
    case AIR780E_MQTT_STEP_MSGSET:
        written = snprintf(destination, capacity, "AT+MQTTMSGSET=1");
        break;
    case AIR780E_MQTT_STEP_MIPSTART:
        written = snprintf(destination, capacity, "AT+MIPSTART=\"%s\",%lu", modem->mqtt_config.host,
                           (unsigned long)modem->mqtt_config.port);
        break;
    case AIR780E_MQTT_STEP_MCONNECT:
        written = snprintf(destination, capacity, "AT+MCONNECT=%u,%lu",
                           modem->mqtt_config.clean_session ? 1U : 0U,
                           (unsigned long)modem->mqtt_config.keepalive_s);
        break;
    case AIR780E_MQTT_STEP_MSUB:
        written = snprintf(destination, capacity, "AT+MSUB=\"%s\",%u", modem->mqtt_topic,
                           (unsigned int)modem->mqtt_qos);
        break;
    case AIR780E_MQTT_STEP_MPUBEX:
        written = snprintf(destination, capacity, "AT+MPUBEX=\"%s\",%u,%u,%lu", modem->mqtt_topic,
                           (unsigned int)modem->mqtt_qos, modem->mqtt_retain ? 1U : 0U,
                           (unsigned long)modem->mqtt_payload_length);
        break;
    case AIR780E_MQTT_STEP_MDISCONNECT:
        written = snprintf(destination, capacity, "AT+MDISCONNECT");
        break;
    case AIR780E_MQTT_STEP_MIPCLOSE:
        written = snprintf(destination, capacity, "AT+MIPCLOSE");
        break;
    default:
        break;
    }

    if (written <= 0 || (size_t)written >= capacity) {
        return false;
    }
    *length = (size_t)written;
    return true;
}

/**
 * @brief 为当前 MQTT 步骤设置 30 秒超时、成功/失败标记行及 MPUBEX 的 '>' 负载。
 * @param modem 传当前 Air780E 对象指针；调用前 modem->request 的通用字段已填好。
 * @return 无
 * @par 示例
 * @code
 * if (!modem->started) {
 *     Air780E_PrepareMqttRequest(modem);
 * }
 * @endcode
 */
static void Air780E_PrepareMqttRequest(Air780E *modem)
{
    modem->request.timeout_ms = 30000U;

    switch (modem->mqtt_step) {
    case AIR780E_MQTT_STEP_MIPSTART:
        modem->request.success_tokens[0] = "OK";
        modem->request.success_tokens[1] = "CONNECT OK";
        modem->request.success_token_count = 2U;
        modem->request.failure_tokens[0] = "CONNECT FAIL";
        modem->request.failure_tokens[1] = "ALREADY CONNECT";
        modem->request.failure_tokens[2] = "CLOSED";
        modem->request.failure_token_count = 3U;
        break;
    case AIR780E_MQTT_STEP_MCONNECT:
        modem->request.success_tokens[0] = "OK";
        modem->request.success_tokens[1] = "CONNACK OK";
        modem->request.success_token_count = 2U;
        modem->request.failure_tokens[0] = "CONNACK FAIL";
        modem->request.failure_tokens[1] = "CLOSED";
        modem->request.failure_token_count = 2U;
        break;
    case AIR780E_MQTT_STEP_MSUB:
        modem->request.success_tokens[0] = "OK";
        modem->request.success_tokens[1] = "SUBACK";
        modem->request.success_token_count = 2U;
        modem->request.failure_tokens[0] = "SUBACK FAIL";
        modem->request.failure_tokens[1] = "CLOSED";
        modem->request.failure_token_count = 2U;
        break;
    case AIR780E_MQTT_STEP_MPUBEX:
        modem->request.prompt_byte = (uint8_t)'>';
        modem->request.payload = modem->mqtt_payload;
        modem->request.payload_length = modem->mqtt_payload_length;
        if (modem->mqtt_qos == 1U) {
            modem->request.success_tokens[0] = "OK";
            modem->request.success_tokens[1] = "PUBACK";
            modem->request.success_token_count = 2U;
        } else if (modem->mqtt_qos == 2U) {
            modem->request.success_tokens[0] = "OK";
            modem->request.success_tokens[1] = "PUBREC";
            modem->request.success_tokens[2] = "PUBCOMP";
            modem->request.success_token_count = 3U;
        }
        modem->request.failure_tokens[0] = "PUBACK FAIL";
        modem->request.failure_tokens[1] = "PUBREC FAIL";
        modem->request.failure_tokens[2] = "CLOSED";
        modem->request.failure_token_count = 3U;
        break;
    default:
        break;
    }
}

/**
 * @brief MQTT 命令完成处理：成功则进入下一步骤或结束操作，失败则结束操作。
 * @param modem  传当前 Air780E 对象指针（内部调用时就是公开接口收到的 modem）。
 * @param result 传 CommandComplete 收到的 ATClient 结果。
 * @return 无
 * @par 示例
 * @code
 * Air780E_MqttCommandComplete(modem, ATCLIENT_RESULT_OK);
 * @endcode
 */
static void Air780E_MqttCommandComplete(Air780E *modem, ATClient_Result result)
{
    if (result != ATCLIENT_RESULT_OK) {
        Air780E_FinishMqttOperation(modem, Air780E_MapAtResult(result));
        return;
    }

    switch (modem->mqtt_step) {
    case AIR780E_MQTT_STEP_MCONFIG:
        modem->mqtt_step = AIR780E_MQTT_STEP_MSGSET;
        modem->command_pending = true;
        break;
    case AIR780E_MQTT_STEP_MSGSET:
        modem->mqtt_step = AIR780E_MQTT_STEP_MIPSTART;
        modem->command_pending = true;
        break;
    case AIR780E_MQTT_STEP_MIPSTART:
        modem->mqtt_step = AIR780E_MQTT_STEP_MCONNECT;
        modem->command_pending = true;
        Air780E_SetState(modem, AIR780E_STATE_MQTT_AUTHENTICATING);
        break;
    case AIR780E_MQTT_STEP_MCONNECT:
        modem->status.mqtt_online = true;
        modem->mqtt_reconnect_index = 0U;
        modem->mqtt_reconnect_waiting = false;
        Air780E_SetState(modem, AIR780E_STATE_MQTT_ONLINE);
        Air780E_FinishMqttOperation(modem, AIR780E_RESULT_OK);
        break;
    case AIR780E_MQTT_STEP_MSUB:
    case AIR780E_MQTT_STEP_MPUBEX:
        Air780E_FinishMqttOperation(modem, AIR780E_RESULT_OK);
        break;
    case AIR780E_MQTT_STEP_MDISCONNECT:
        modem->mqtt_step = AIR780E_MQTT_STEP_MIPCLOSE;
        modem->command_pending = true;
        break;
    case AIR780E_MQTT_STEP_MIPCLOSE:
        modem->status.mqtt_online = false;
        Air780E_SetState(modem, AIR780E_STATE_OFFLINE);
        Air780E_FinishMqttOperation(modem, AIR780E_RESULT_OK);
        break;
    default:
        Air780E_FinishMqttOperation(modem, AIR780E_RESULT_AT_ERROR);
        break;
    }
}

/**
 * @brief 结束当前 MQTT 操作：清理状态、必要时安排自动重连，并调用 on_mqtt_operation_complete。
 * @param modem  传当前 Air780E 对象指针（内部调用时就是公开接口收到的 modem）。
 * @param result 传本次 MQTT 操作的最终结果，如 AIR780E_RESULT_OK。
 * @return 无
 * @par 示例
 * @code
 * Air780E_FinishMqttOperation(modem, AIR780E_RESULT_TIMEOUT);
 * @endcode
 */
static void Air780E_FinishMqttOperation(Air780E *modem, Air780E_Result result)
{
    Air780E_MqttOperation operation = modem->mqtt_operation;
    bool should_reconnect = result != AIR780E_RESULT_OK &&
                            operation == AIR780E_MQTT_OPERATION_CONNECT && modem->mqtt_configured &&
                            modem->mqtt_config.auto_reconnect && !modem->mqtt_user_disconnect;

    modem->command_pending = false;
    modem->command_active = false;
    modem->mqtt_operation = AIR780E_MQTT_OPERATION_NONE;
    modem->mqtt_step = AIR780E_MQTT_STEP_IDLE;
    modem->mqtt_topic = NULL;
    modem->mqtt_payload = NULL;
    modem->mqtt_payload_length = 0U;
    modem->status.last_mqtt_operation = operation;
    modem->status.last_result = result;
    if (should_reconnect) {
        modem->status.mqtt_online = false;
        Air780E_ScheduleMqttReconnect(modem);
    } else if (result != AIR780E_RESULT_OK && operation == AIR780E_MQTT_OPERATION_CONNECT) {
        modem->status.mqtt_online = false;
        Air780E_SetState(modem, AIR780E_STATE_ERROR);
    }
    if (modem->callbacks.on_mqtt_operation_complete != NULL) {
        modem->callbacks.on_mqtt_operation_complete(modem->callback_context, operation, result);
    }
}

/**
 * @brief 返回启动阶段对应的 AT 命令字符串。
 * @param stage 传启动阶段，如 AIR780E_STAGE_CPIN。
 * @return 命令字符串；IDLE/COMPLETE 或越界时返回 NULL。
 * @par 示例
 * @code
 * const char *cmd = Air780E_CommandForStage(AIR780E_STAGE_CPIN);   // "AT+CPIN?"
 * @endcode
 */
static const char *Air780E_CommandForStage(Air780E_Stage stage)
{
    static const char *const commands[] = {
        NULL,      "AT",       "ATE0",      "AT+CMEE=2", "ATI", "AT+CGMR",
        "AT+CGSN", "AT+CPIN?", "AT+CEREG?", "AT+CGATT?", NULL,
    };

    return stage <= AIR780E_STAGE_COMPLETE ? commands[stage] : NULL;
}

/**
 * @brief 进入下一个启动阶段并更新对外状态；到达 COMPLETE 时结束启动。
 * @param modem 传当前 Air780E 对象指针（内部调用时就是公开接口收到的 modem）。
 * @return 无
 * @par 示例
 * @code
 * modem->stage_retries = 0U;
 * Air780E_AdvanceStage(modem);
 * @endcode
 */
static void Air780E_AdvanceStage(Air780E *modem)
{
    modem->stage = (Air780E_Stage)(modem->stage + 1);
    if (modem->stage == AIR780E_STAGE_CPIN) {
        Air780E_SetState(modem, AIR780E_STATE_SIM_WAIT);
    } else if (modem->stage == AIR780E_STAGE_CEREG || modem->stage == AIR780E_STAGE_CGATT) {
        Air780E_SetState(modem, AIR780E_STATE_NETWORK_WAIT);
    }

    if (modem->stage == AIR780E_STAGE_COMPLETE) {
        Air780E_FinishStartup(modem, AIR780E_RESULT_OK);
    } else {
        modem->command_pending = true;
    }
}

/**
 * @brief 修改对外状态，变化时调用 on_state_changed。
 * @param modem 传当前 Air780E 对象指针（内部调用时就是公开接口收到的 modem）。
 * @param state 传要切换到的状态，如 AIR780E_STATE_NETWORK_WAIT。
 * @return 无
 * @par 示例
 * @code
 * Air780E_SetState(modem, AIR780E_STATE_NETWORK_WAIT);
 * @endcode
 */
static void Air780E_SetState(Air780E *modem, Air780E_State state)
{
    if (modem->state == state) {
        return;
    }
    modem->state = state;
    if (modem->callbacks.on_state_changed != NULL) {
        modem->callbacks.on_state_changed(modem->callback_context, state);
    }
}

/**
 * @brief 结束启动流程：成功回到 OFFLINE（可连 MQTT），失败进入 ERROR，并通知 on_startup_complete。
 * @param modem  传当前 Air780E 对象指针（内部调用时就是公开接口收到的 modem）。
 * @param result 传启动结果，AIR780E_RESULT_OK 表示成功。
 * @return 无
 * @par 示例
 * @code
 * Air780E_FinishStartup(modem, AIR780E_RESULT_PARSE_ERROR);
 * @endcode
 */
static void Air780E_FinishStartup(Air780E *modem, Air780E_Result result)
{
    modem->started = false;
    modem->command_pending = false;
    modem->command_active = false;
    modem->retry_waiting = false;
    modem->status.last_result = result;
    Air780E_SetState(modem,
                     result == AIR780E_RESULT_OK ? AIR780E_STATE_OFFLINE : AIR780E_STATE_ERROR);
    if (modem->callbacks.on_startup_complete != NULL) {
        modem->callbacks.on_startup_complete(modem->callback_context, result);
    }
}

/**
 * @brief 安排当前启动阶段在 1 秒后重试；超过 3 次则以 result 结束启动。
 * @param modem  传当前 Air780E 对象指针（内部调用时就是公开接口收到的 modem）。
 * @param result 传本次失败的原因，如 AIR780E_RESULT_NOT_READY。
 * @return 无
 * @par 示例
 * @code
 * Air780E_ScheduleRetry(modem, AIR780E_RESULT_NOT_READY);
 * @endcode
 */
static void Air780E_ScheduleRetry(Air780E *modem, Air780E_Result result)
{
    modem->status.last_result = result;
    if (modem->stage_retries >= AIR780E_MAX_STAGE_RETRIES) {
        Air780E_FinishStartup(modem, result);
        return;
    }

    ++modem->stage_retries;
    ++modem->status.retry_count;
    modem->retry_due_ms = ATClient_NowMs(modem->at) + AIR780E_RETRY_DELAY_MS;
    modem->retry_waiting = true;
}

/**
 * @brief 把 ATClient 结果码映射为 Air780E 结果码。
 * @param result 传 ATClient 返回的结果码。
 * @return 对应的 Air780E_Result。
 * @par 示例
 * @code
 * Air780E_Result r = Air780E_MapAtResult(ATCLIENT_RESULT_TIMEOUT);   // AIR780E_RESULT_TIMEOUT
 * @endcode
 */
static Air780E_Result Air780E_MapAtResult(ATClient_Result result)
{
    if (result == ATCLIENT_RESULT_TIMEOUT) {
        return AIR780E_RESULT_TIMEOUT;
    }
    if (result == ATCLIENT_RESULT_INVALID_ARGUMENT) {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    return result == ATCLIENT_RESULT_OK ? AIR780E_RESULT_OK : AIR780E_RESULT_AT_ERROR;
}

/**
 * @brief 把长度已知的字段复制为以 '\0' 结尾的字符串。
 * @param destination 输出：传目标字符数组，如 modem->status.model。
 * @param capacity    传目标数组容量，一般用 sizeof(modem->status.model)。
 * @param source      传源数据（不要求以 '\0' 结尾）。
 * @param length      传要复制的字节数。
 * @return true 成功；false 长度为 0 或放不下。
 * @par 示例
 * @code
 * (void)Air780E_CopyField(modem->status.model, sizeof(modem->status.model), line, length);
 * @endcode
 */
static bool Air780E_CopyField(char *destination, size_t capacity, const char *source, size_t length)
{
    if (length == 0U || length >= capacity) {
        return false;
    }
    memcpy(destination, source, length);
    destination[length] = '\0';
    return true;
}

/**
 * @brief 在 [*cursor, end) 范围内跳过前导空格并解析无符号十进制数。
 * @param cursor 传字符游标变量的地址（&cursor）；成功时前移到数字之后。
 * @param end    传解析范围的结束位置（一般为 line + length）。
 * @param value  输出：传 unsigned int 变量的地址，用于接收解析结果。
 * @return true 至少解析到一位数字；false 没有数字。
 * @par 示例
 * @code
 * const char *cursor = line + 5U;
 * unsigned int rssi;
 * (void)Air780E_ParseUnsigned(&cursor, line + length, &rssi);
 * @endcode
 */
static bool Air780E_ParseUnsigned(const char **cursor, const char *end, unsigned int *value)
{
    unsigned int parsed = 0U;
    bool has_digit = false;

    while (*cursor < end && **cursor == ' ') {
        ++*cursor;
    }
    while (*cursor < end && **cursor >= '0' && **cursor <= '9') {
        has_digit = true;
        parsed = parsed * 10U + (unsigned int)(**cursor - '0');
        ++*cursor;
    }
    if (!has_digit) {
        return false;
    }
    *value = parsed;
    return true;
}

/**
 * @brief 判断 [cursor, end) 范围内是否只剩空格。
 * @param cursor 传当前解析位置。
 * @param end    传解析范围的结束位置（一般为 line + length）。
 * @return true 只剩空格或为空；false 还有其他字符。
 * @par 示例
 * @code
 * if (!Air780E_OnlySpacesRemain(cursor, end)) {
 *     return false;
 * }
 * @endcode
 */
static bool Air780E_OnlySpacesRemain(const char *cursor, const char *end)
{
    while (cursor < end && *cursor == ' ') {
        ++cursor;
    }
    return cursor == end;
}

/**
 * @brief 检查 MQTT 字符串字段能否放进 AT 命令的引号参数（长度、控制字符、双引号）。
 * @param value       传要检查的字符串，可以是 NULL（视为非法）。
 * @param allow_empty 传 true 允许空串（用户名/密码），传 false 不允许（主机/ClientID/主题）。
 * @return true 合法；false 为 NULL、超长或含非法字符。
 * @par 示例
 * @code
 * if (!Air780E_MqttFieldValid(config->username, true)) {
 *     return AIR780E_RESULT_INVALID_ARGUMENT;
 * }
 * @endcode
 */
static bool Air780E_MqttFieldValid(const char *value, bool allow_empty)
{
    size_t length = 0U;

    if (value == NULL) {
        return false;
    }
    while (value[length] != '\0') {
        unsigned char byte = (unsigned char)value[length];

        if (length >= AIR780E_MQTT_FIELD_MAX || byte < 0x20U || byte == 0x7FU ||
            byte == (unsigned char)'\"') {
            return false;
        }
        ++length;
    }
    return allow_empty || length > 0U;
}

/**
 * @brief 收到“消息已缓存”通知后，以 URGENT 优先级提交 AT+MQTTMSGGET 读取缓存消息。
 * @param modem 传当前 Air780E 对象指针（内部调用时就是公开接口收到的 modem）。
 * @return 无
 * @par 示例
 * @code
 * Air780E_SubmitCachedGet(modem);   // 在 Air780E_Process() 开头调用
 * @endcode
 */
static void Air780E_SubmitCachedGet(Air780E *modem)
{
    ATClient_Result submit_result;

    if (modem->mqtt_cache_notices == 0U || modem->mqtt_get_queued) {
        return;
    }

    memset(&modem->mqtt_get_request, 0, sizeof(modem->mqtt_get_request));
    modem->mqtt_get_request.build_command = Air780E_BuildMqttGet;
    modem->mqtt_get_request.command_context = modem;
    modem->mqtt_get_request.timeout_ms = 10000U;
    modem->mqtt_get_request.priority = ATCLIENT_PRIORITY_URGENT;
    modem->mqtt_get_request.on_complete = Air780E_MqttGetComplete;
    modem->mqtt_get_request.callback_context = modem;
    submit_result = ATClient_Submit(modem->at, &modem->mqtt_get_request);
    if (submit_result == ATCLIENT_RESULT_OK) {
        modem->mqtt_get_queued = true;
        modem->mqtt_cache_notices = 0U;
    }
}

/**
 * @brief AT+MQTTMSGGET 命令构造回调。
 * @param context     由 ATClient 传入：command_context（未使用）。
 * @param destination 由 ATClient 传入：命令写入的缓冲区。
 * @param capacity    由 ATClient 传入：destination 的容量。
 * @param length      输出：写入命令长度。
 * @return true 成功；false 缓冲区不足。
 * @par 示例
 * @code
 * modem->mqtt_get_request.build_command = Air780E_BuildMqttGet;
 * @endcode
 */
static bool Air780E_BuildMqttGet(void *context, char *destination, size_t capacity, size_t *length)
{
    static const char command[] = "AT+MQTTMSGGET";

    (void)context;
    if (capacity < sizeof(command) - 1U) {
        return false;
    }
    memcpy(destination, command, sizeof(command) - 1U);
    *length = sizeof(command) - 1U;
    return true;
}

/**
 * @brief AT+MQTTMSGGET 完成回调：记录结果并以 CACHE_GET 操作通知 on_mqtt_operation_complete。
 * @param context 由 ATClient 传入：注册/提交时给的 Air780E 对象指针。
 * @param result  由 ATClient 传入：AT+MQTTMSGGET 的完成结果。
 * @return 无
 * @par 示例
 * @code
 * modem->mqtt_get_request.on_complete = Air780E_MqttGetComplete;
 * @endcode
 */
static void Air780E_MqttGetComplete(void *context, ATClient_Result result)
{
    Air780E *modem = (Air780E *)context;
    Air780E_Result mapped = Air780E_MapAtResult(result);

    modem->mqtt_get_queued = false;
    modem->status.last_mqtt_operation = AIR780E_MQTT_OPERATION_CACHE_GET;
    modem->status.last_result = mapped;
    if (modem->callbacks.on_mqtt_operation_complete != NULL) {
        modem->callbacks.on_mqtt_operation_complete(modem->callback_context,
                                                    AIR780E_MQTT_OPERATION_CACHE_GET, mapped);
    }
}

/**
 * @brief 不带逗号的 "+MSUB:" URC（消息已进入模组缓存）计数，最多累计 4 条，超出计入覆盖风险。
 * @param context 由 ATClient 传入：注册/提交时给的 Air780E 对象指针。
 * @param line    由 ATClient 传入：URC 行，如 "+MSUB: 1"。
 * @param length  由 ATClient 传入：line 的字节数。
 * @return 无
 * @par 示例
 * @code
 * (void)ATClient_RegisterUrc(at, "+MSUB:", Air780E_CacheUrc, modem);
 * @endcode
 */
static void Air780E_CacheUrc(void *context, const char *line, size_t length)
{
    Air780E *modem = (Air780E *)context;

    if (memchr(line, ',', length) != NULL) {
        return;
    }
    if (modem->mqtt_cache_notices < 4U) {
        ++modem->mqtt_cache_notices;
    } else {
        ++modem->status.mqtt_cache_overwrite_risk;
    }
}

/**
 * @brief "+MSUB: <topic>,<len>," 帧头探测：校验主题长度并解析负载长度。
 * @param context        由 ATClient 传入：注册时的 context（未使用）。
 * @param header         由 ATClient 传入：当前已收到的帧头字节。
 * @param header_length  由 ATClient 传入：header 的字节数。
 * @param payload_length 输出：返回 READY 时写入负载字节数。
 * @return NO_MATCH / NEED_MORE / READY / ERROR。
 * @par 示例
 * @code
 * frame_registration.prefix = "+MSUB:";
 * frame_registration.probe = Air780E_MqttFrameProbe;
 * @endcode
 */
static ATClient_FrameProbeResult Air780E_MqttFrameProbe(void *context, const uint8_t *header,
                                                        size_t header_length,
                                                        size_t *payload_length)
{
    static const char prefix[] = "+MSUB:";
    const size_t prefix_length = sizeof(prefix) - 1U;
    size_t index;
    size_t topic_start;
    size_t topic_end = 0U;
    size_t value = 0U;
    bool has_length_digit = false;

    (void)context;
    if (header_length <= prefix_length) {
        bool is_prefix = memcmp(header, prefix, header_length) == 0;
        return is_prefix ? ATCLIENT_FRAME_NEED_MORE : ATCLIENT_FRAME_NO_MATCH;
    }
    if (memcmp(header, prefix, prefix_length) != 0) {
        return ATCLIENT_FRAME_NO_MATCH;
    }

    topic_start = prefix_length;
    while (topic_start < header_length && header[topic_start] == (uint8_t)' ') {
        ++topic_start;
    }
    for (index = topic_start; index < header_length; ++index) {
        if (topic_end == 0U) {
            if (header[index] == (uint8_t)',') {
                if (index == topic_start || index - topic_start > AIR780E_MQTT_FIELD_MAX) {
                    return ATCLIENT_FRAME_ERROR;
                }
                topic_end = index;
                continue;
            }
            if (index - topic_start >= AIR780E_MQTT_FIELD_MAX) {
                return ATCLIENT_FRAME_ERROR;
            }
            continue;
        }

        if (header[index] == (uint8_t)',') {
            if (!has_length_digit || index + 1U != header_length) {
                return ATCLIENT_FRAME_ERROR;
            }
            *payload_length = value;
            bool length_ok = value <= ATCLIENT_FRAME_PAYLOAD_MAX;
            return length_ok ? ATCLIENT_FRAME_READY : ATCLIENT_FRAME_ERROR;
        }
        if (header[index] < (uint8_t)'0' || header[index] > (uint8_t)'9') {
            return ATCLIENT_FRAME_ERROR;
        }
        has_length_digit = true;
        value = value * 10U + (size_t)(header[index] - (uint8_t)'0');
        if (value > ATCLIENT_FRAME_PAYLOAD_MAX) {
            return ATCLIENT_FRAME_ERROR;
        }
    }
    return ATCLIENT_FRAME_NEED_MORE;
}

/**
 * @brief 订阅消息开始：从帧头中取出主题并调用 on_mqtt_message_begin。
 * @param context        由 ATClient 传入：注册/提交时给的 Air780E 对象指针。
 * @param header         由 ATClient 传入：完整帧头，如 "+MSUB: \"topic\",5,"。
 * @param header_length  由 ATClient 传入：header 的字节数。
 * @param payload_length 由 ATClient 传入：消息负载字节数。
 * @return 无
 * @par 示例
 * @code
 * frame_registration.on_begin = Air780E_MqttFrameBegin;
 * @endcode
 */
static void Air780E_MqttFrameBegin(void *context, const uint8_t *header, size_t header_length,
                                   size_t payload_length)
{
    Air780E *modem = (Air780E *)context;
    size_t start = sizeof("+MSUB:") - 1U;
    size_t end = start;

    while (start < header_length && header[start] == (uint8_t)' ') {
        ++start;
    }
    end = start;
    while (end < header_length && header[end] != (uint8_t)',') {
        ++end;
    }
    memcpy(modem->mqtt_rx_topic, &header[start], end - start);
    modem->mqtt_rx_topic[end - start] = '\0';
    if (modem->callbacks.on_mqtt_message_begin != NULL) {
        modem->callbacks.on_mqtt_message_begin(modem->callback_context, modem->mqtt_rx_topic,
                                               payload_length);
    }
}

/**
 * @brief 订阅消息负载片段，转发给 on_mqtt_message_data。
 * @param context 由 ATClient 传入：注册/提交时给的 Air780E 对象指针。
 * @param data    由 ATClient 传入：本次收到的负载片段。
 * @param length  由 ATClient 传入：data 的字节数。
 * @return 无
 * @par 示例
 * @code
 * frame_registration.on_data = Air780E_MqttFrameData;
 * @endcode
 */
static void Air780E_MqttFrameData(void *context, const uint8_t *data, size_t length)
{
    Air780E *modem = (Air780E *)context;

    if (modem->callbacks.on_mqtt_message_data != NULL) {
        modem->callbacks.on_mqtt_message_data(modem->callback_context, data, length);
    }
}

/**
 * @brief 订阅消息结束：成功时累加消息计数，并调用 on_mqtt_message_end。
 * @param context 由 ATClient 传入：注册/提交时给的 Air780E 对象指针。
 * @param result  由 ATClient 传入：ATCLIENT_RESULT_OK 表示收齐。
 * @return 无
 * @par 示例
 * @code
 * frame_registration.on_end = Air780E_MqttFrameEnd;
 * @endcode
 */
static void Air780E_MqttFrameEnd(void *context, ATClient_Result result)
{
    Air780E *modem = (Air780E *)context;
    Air780E_Result mapped = Air780E_MapAtResult(result);

    if (result == ATCLIENT_RESULT_OK) {
        ++modem->status.mqtt_message_count;
    }
    if (modem->callbacks.on_mqtt_message_end != NULL) {
        modem->callbacks.on_mqtt_message_end(modem->callback_context, mapped);
    }
}

/**
 * @brief "CLOSED" URC：TCP 连接断开，非主动断开时安排自动重连。
 * @param context 由 ATClient 传入：注册/提交时给的 Air780E 对象指针。
 * @param line    由 ATClient 传入：URC 行（未使用）。
 * @param length  由 ATClient 传入：行长度（未使用）。
 * @return 无
 * @par 示例
 * @code
 * (void)ATClient_RegisterUrc(at, "CLOSED", Air780E_ClosedUrc, modem);
 * @endcode
 */
static void Air780E_ClosedUrc(void *context, const char *line, size_t length)
{
    Air780E *modem = (Air780E *)context;

    (void)line;
    (void)length;
    modem->status.mqtt_online = false;
    if (modem->mqtt_user_disconnect) {
        Air780E_SetState(modem, AIR780E_STATE_OFFLINE);
        return;
    }
    Air780E_ScheduleMqttReconnect(modem);
}

/**
 * @brief "+MQTTSTATU: <n>" URC：0=断开（触发重连）、1=认证中、2=在线。
 * @param context 由 ATClient 传入：注册/提交时给的 Air780E 对象指针。
 * @param line    由 ATClient 传入：URC 行，如 "+MQTTSTATU: 2"。
 * @param length  由 ATClient 传入：line 的字节数。
 * @return 无
 * @par 示例
 * @code
 * (void)ATClient_RegisterUrc(at, "+MQTTSTATU:", Air780E_MqttStatusUrc, modem);
 * @endcode
 */
static void Air780E_MqttStatusUrc(void *context, const char *line, size_t length)
{
    Air780E *modem = (Air780E *)context;
    const size_t prefix_length = sizeof("+MQTTSTATU:") - 1U;
    const char *cursor = line + prefix_length;
    const char *end = line + length;
    unsigned int value;

    if (length <= prefix_length || !Air780E_ParseUnsigned(&cursor, end, &value) ||
        !Air780E_OnlySpacesRemain(cursor, end) || value > 2U) {
        ++modem->status.parse_error_count;
        return;
    }

    if (value == 0U) {
        modem->status.mqtt_online = false;
        if (!modem->mqtt_user_disconnect) {
            Air780E_ScheduleMqttReconnect(modem);
        }
    } else if (value == 1U) {
        modem->status.mqtt_online = false;
        Air780E_SetState(modem, AIR780E_STATE_MQTT_AUTHENTICATING);
    } else {
        modem->status.mqtt_online = true;
        modem->mqtt_reconnect_index = 0U;
        modem->mqtt_reconnect_waiting = false;
        Air780E_SetState(modem, AIR780E_STATE_MQTT_ONLINE);
    }
}

/**
 * @brief 按 1/2/4/8/16/30 秒指数退避安排下一次自动重连；未开启自动重连时转为 OFFLINE。
 * @param modem 传当前 Air780E 对象指针（内部调用时就是公开接口收到的 modem）。
 * @return 无
 * @par 示例
 * @code
 * modem->status.mqtt_online = false;
 * Air780E_ScheduleMqttReconnect(modem);
 * @endcode
 */
static void Air780E_ScheduleMqttReconnect(Air780E *modem)
{
    static const uint32_t delays_ms[] = {
        1000U, 2000U, 4000U, 8000U, 16000U, 30000U,
    };
    uint8_t index;

    if (!modem->mqtt_configured || !modem->mqtt_config.auto_reconnect ||
        modem->mqtt_user_disconnect) {
        Air780E_SetState(modem, AIR780E_STATE_OFFLINE);
        return;
    }
    if (modem->mqtt_reconnect_waiting) {
        return;
    }

    index = modem->mqtt_reconnect_index;
    if (index >= sizeof(delays_ms) / sizeof(delays_ms[0])) {
        index = (uint8_t)(sizeof(delays_ms) / sizeof(delays_ms[0]) - 1U);
    }
    modem->mqtt_reconnect_due_ms = ATClient_NowMs(modem->at) + delays_ms[index];
    if (modem->mqtt_reconnect_index < sizeof(delays_ms) / sizeof(delays_ms[0]) - 1U) {
        ++modem->mqtt_reconnect_index;
    }
    modem->mqtt_reconnect_waiting = true;
    Air780E_SetState(modem, AIR780E_STATE_RECONNECT_WAIT);
}

/**
 * @brief 解析 "+CEREG: [<n>,]<stat>" 并保存注册状态。
 * @param modem  传当前 Air780E 对象指针（内部调用时就是公开接口收到的 modem）。
 * @param line   传完整的响应行，如 "+CEREG: 0,1"。
 * @param length 传 line 的字节数。
 * @return true 解析成功；false 格式错误或状态值越界。
 * @par 示例
 * @code
 * modem->stage_value_valid = Air780E_ParseCereg(modem, "+CEREG: 0,1", 11U);
 * @endcode
 */
static bool Air780E_ParseCereg(Air780E *modem, const char *line, size_t length)
{
    const char *cursor = line + 7U;
    const char *end = line + length;
    unsigned int first;
    unsigned int second;

    if (!Air780E_ParseUnsigned(&cursor, end, &first)) {
        return false;
    }
    while (cursor < end && *cursor == ' ') {
        ++cursor;
    }
    if (cursor < end && *cursor == ',') {
        ++cursor;
        if (!Air780E_ParseUnsigned(&cursor, end, &second)) {
            return false;
        }
        first = second;
    }
    if (!Air780E_OnlySpacesRemain(cursor, end) || first > 5U) {
        return false;
    }
    modem->status.registration = (uint8_t)first;
    return true;
}

/**
 * @brief 解析 "+CGATT: <0|1>" 并保存分组域附着状态。
 * @param modem  传当前 Air780E 对象指针（内部调用时就是公开接口收到的 modem）。
 * @param line   传完整的响应行，如 "+CGATT: 1"。
 * @param length 传 line 的字节数。
 * @return true 解析成功；false 格式错误。
 * @par 示例
 * @code
 * modem->stage_value_valid = Air780E_ParseCgatt(modem, "+CGATT: 1", 9U);
 * @endcode
 */
static bool Air780E_ParseCgatt(Air780E *modem, const char *line, size_t length)
{
    const char *cursor = line + 7U;
    const char *end = line + length;
    unsigned int value;

    if (!Air780E_ParseUnsigned(&cursor, end, &value) || !Air780E_OnlySpacesRemain(cursor, end) ||
        value > 1U) {
        return false;
    }
    modem->status.packet_attached = value == 1U;
    return true;
}

/**
 * @brief 解析 "+CSQ: <rssi>,<ber>"，并把 rssi 换算为 dBm（-113 + 2*rssi）。
 * @param modem  传当前 Air780E 对象指针（内部调用时就是公开接口收到的 modem）。
 * @param line   传完整的响应行，如 "+CSQ: 20,99"。
 * @param length 传 line 的字节数。
 * @return true 解析成功；false 格式错误或数值越界。
 * @par 示例
 * @code
 * (void)Air780E_ParseCsq(modem, "+CSQ: 20,99", 11U);   // rssi_dbm = -73
 * @endcode
 */
static bool Air780E_ParseCsq(Air780E *modem, const char *line, size_t length)
{
    const char *cursor = line + 5U;
    const char *end = line + length;
    unsigned int rssi;
    unsigned int ber;

    if (!Air780E_ParseUnsigned(&cursor, end, &rssi) || cursor >= end || *cursor != ',') {
        return false;
    }
    ++cursor;
    if (!Air780E_ParseUnsigned(&cursor, end, &ber) || !Air780E_OnlySpacesRemain(cursor, end) ||
        (rssi > 31U && rssi != 99U) || (ber > 7U && ber != 99U)) {
        return false;
    }

    modem->status.rssi = (uint8_t)rssi;
    modem->status.ber = (uint8_t)ber;
    modem->status.rssi_known = rssi != 99U;
    modem->status.rssi_dbm = modem->status.rssi_known ? (int16_t)((int)rssi * 2 - 113) : 0;
    return true;
}
