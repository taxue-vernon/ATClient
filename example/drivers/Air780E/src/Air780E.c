#include "Air780E.h"

#include <stdio.h>
#include <string.h>

#define AIR780E_COMMAND_TIMEOUT_MS  5000U
#define AIR780E_RETRY_DELAY_MS      1000U
#define AIR780E_MAX_STAGE_RETRIES   3U

static bool Air780E_BuildCommand(void *context,
                                 char *destination,
                                 size_t capacity,
                                 size_t *length);
static void Air780E_HandleLine(void *context,
                               const char *line,
                               size_t length);
static void Air780E_CommandComplete(void *context, ATClient_Result result);
static const char *Air780E_CommandForStage(Air780E_Stage stage);
static bool Air780E_BuildMqttCommand(Air780E *modem,
                                     char *destination,
                                     size_t capacity,
                                     size_t *length);
static void Air780E_PrepareMqttRequest(Air780E *modem);
static void Air780E_MqttCommandComplete(Air780E *modem,
                                        ATClient_Result result);
static void Air780E_FinishMqttOperation(Air780E *modem,
                                        Air780E_Result result);
static void Air780E_AdvanceStage(Air780E *modem);
static void Air780E_SetState(Air780E *modem, Air780E_State state);
static void Air780E_FinishStartup(Air780E *modem, Air780E_Result result);
static void Air780E_ScheduleRetry(Air780E *modem, Air780E_Result result);
static Air780E_Result Air780E_MapAtResult(ATClient_Result result);
static bool Air780E_CopyField(char *destination,
                              size_t capacity,
                              const char *source,
                              size_t length);
static bool Air780E_ParseUnsigned(const char **cursor,
                                  const char *end,
                                  unsigned int *value);
static bool Air780E_ParseCereg(Air780E *modem,
                               const char *line,
                               size_t length);
static bool Air780E_ParseCgatt(Air780E *modem,
                               const char *line,
                               size_t length);
static bool Air780E_ParseCsq(Air780E *modem,
                             const char *line,
                             size_t length);
static bool Air780E_MqttFieldValid(const char *value, bool allow_empty);
static void Air780E_SubmitCachedGet(Air780E *modem);
static bool Air780E_BuildMqttGet(void *context,
                                 char *destination,
                                 size_t capacity,
                                 size_t *length);
static void Air780E_MqttGetComplete(void *context, ATClient_Result result);
static void Air780E_CacheUrc(void *context,
                             const char *line,
                             size_t length);
static ATClient_FrameProbeResult Air780E_MqttFrameProbe(
    void *context,
    const uint8_t *header,
    size_t header_length,
    size_t *payload_length);
static void Air780E_MqttFrameBegin(void *context,
                                    const uint8_t *header,
                                    size_t header_length,
                                    size_t payload_length);
static void Air780E_MqttFrameData(void *context,
                                  const uint8_t *data,
                                  size_t length);
static void Air780E_MqttFrameEnd(void *context, ATClient_Result result);
static void Air780E_ClosedUrc(void *context,
                              const char *line,
                              size_t length);
static void Air780E_MqttStatusUrc(void *context,
                                  const char *line,
                                  size_t length);
static void Air780E_ScheduleMqttReconnect(Air780E *modem);

bool Air780E_Init(Air780E *modem,
                  ATClient *at,
                  const Air780E_Callbacks *callbacks,
                  void *callback_context)
{
    ATClient_LengthFrameRegistration frame_registration;

    if (modem == NULL || at == NULL)
    {
        return false;
    }

    memset(modem, 0, sizeof(*modem));
    modem->at = at;
    if (callbacks != NULL)
    {
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
    return ATClient_RegisterLengthFrame(at, &frame_registration) ==
               ATCLIENT_RESULT_OK &&
           ATClient_RegisterUrc(at, "+MSUB:", Air780E_CacheUrc, modem) ==
               ATCLIENT_RESULT_OK &&
           ATClient_RegisterUrc(at, "CLOSED", Air780E_ClosedUrc, modem) ==
               ATCLIENT_RESULT_OK &&
           ATClient_RegisterUrc(at,
                                "+MQTTSTATU:",
                                Air780E_MqttStatusUrc,
                                modem) == ATCLIENT_RESULT_OK;
}

Air780E_Result Air780E_Start(Air780E *modem)
{
    if (modem == NULL || modem->at == NULL)
    {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    if (modem->started || modem->command_active || modem->command_pending)
    {
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

Air780E_Result Air780E_MqttConfigure(Air780E *modem,
                                     const Air780E_MqttConfig *config)
{
    size_t mconfig_length;
    size_t mipstart_length;

    if (modem == NULL || config == NULL || config->port == 0U ||
        config->port > 65535U || config->keepalive_s == 0U ||
        config->keepalive_s > 65535U ||
        !Air780E_MqttFieldValid(config->host, false) ||
        !Air780E_MqttFieldValid(config->client_id, false) ||
        !Air780E_MqttFieldValid(config->username, true) ||
        !Air780E_MqttFieldValid(config->password, true))
    {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    if (modem->started || modem->command_active || modem->command_pending)
    {
        return AIR780E_RESULT_BUSY;
    }

    mconfig_length = strlen("AT+MCONFIG=\"\",\"\",\"\"") +
                     strlen(config->client_id) + strlen(config->username) +
                     strlen(config->password);
    mipstart_length = strlen("AT+MIPSTART=\"\",65535") +
                      strlen(config->host);
    if (mconfig_length > ATCLIENT_COMMAND_CAPACITY ||
        mipstart_length > ATCLIENT_COMMAND_CAPACITY)
    {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }

    modem->mqtt_config = *config;
    modem->mqtt_configured = true;
    return AIR780E_RESULT_OK;
}

Air780E_Result Air780E_MqttConnect(Air780E *modem)
{
    if (modem == NULL)
    {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    if (!modem->mqtt_configured || !modem->status.packet_attached)
    {
        return AIR780E_RESULT_NOT_READY;
    }
    if (modem->started || modem->mqtt_operation != AIR780E_MQTT_OPERATION_NONE ||
        modem->command_active || modem->command_pending)
    {
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

Air780E_Result Air780E_MqttSubscribe(Air780E *modem,
                                     const char *topic,
                                     uint8_t qos)
{
    if (modem == NULL || qos > 2U ||
        !Air780E_MqttFieldValid(topic, false))
    {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    if (!modem->status.mqtt_online)
    {
        return AIR780E_RESULT_NOT_READY;
    }
    if (modem->started || modem->mqtt_operation != AIR780E_MQTT_OPERATION_NONE ||
        modem->command_active || modem->command_pending)
    {
        return AIR780E_RESULT_BUSY;
    }

    modem->mqtt_topic = topic;
    modem->mqtt_qos = qos;
    modem->mqtt_operation = AIR780E_MQTT_OPERATION_SUBSCRIBE;
    modem->mqtt_step = AIR780E_MQTT_STEP_MSUB;
    modem->command_pending = true;
    return AIR780E_RESULT_OK;
}

Air780E_Result Air780E_MqttPublish(Air780E *modem,
                                   const char *topic,
                                   const uint8_t *payload,
                                   size_t length,
                                   uint8_t qos,
                                   bool retain)
{
    if (modem == NULL || payload == NULL || length == 0U ||
        length > ATCLIENT_FRAME_PAYLOAD_MAX || qos > 2U ||
        !Air780E_MqttFieldValid(topic, false))
    {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    if (!modem->status.mqtt_online)
    {
        return AIR780E_RESULT_NOT_READY;
    }
    if (modem->started || modem->mqtt_operation != AIR780E_MQTT_OPERATION_NONE ||
        modem->command_active || modem->command_pending)
    {
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

Air780E_Result Air780E_MqttDisconnect(Air780E *modem)
{
    if (modem == NULL)
    {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    if (!modem->status.mqtt_online)
    {
        return AIR780E_RESULT_NOT_READY;
    }
    if (modem->started || modem->mqtt_operation != AIR780E_MQTT_OPERATION_NONE ||
        modem->command_active || modem->command_pending)
    {
        return AIR780E_RESULT_BUSY;
    }

    modem->mqtt_user_disconnect = true;
    modem->mqtt_reconnect_waiting = false;
    modem->mqtt_operation = AIR780E_MQTT_OPERATION_DISCONNECT;
    modem->mqtt_step = AIR780E_MQTT_STEP_MDISCONNECT;
    modem->command_pending = true;
    return AIR780E_RESULT_OK;
}

void Air780E_Process(Air780E *modem)
{
    ATClient_Result submit_result;

    if (modem == NULL)
    {
        return;
    }
    Air780E_SubmitCachedGet(modem);
    /* Non-blocking reconnect: compare the injected tick and enqueue work only;
     * never delay or wait here for the modem or network. */
    if (modem->mqtt_reconnect_waiting && !modem->started &&
        modem->mqtt_operation == AIR780E_MQTT_OPERATION_NONE &&
        (int32_t)(ATClient_NowMs(modem->at) -
                  modem->mqtt_reconnect_due_ms) >= 0)
    {
        modem->mqtt_reconnect_waiting = false;
        ++modem->status.mqtt_reconnect_count;
        modem->mqtt_operation = AIR780E_MQTT_OPERATION_CONNECT;
        modem->mqtt_step = AIR780E_MQTT_STEP_MCONFIG;
        modem->command_pending = true;
        Air780E_SetState(modem, AIR780E_STATE_MQTT_TCP_CONNECTING);
    }
    if (modem->command_active)
    {
        return;
    }

    if (!modem->started &&
        modem->mqtt_operation == AIR780E_MQTT_OPERATION_NONE)
    {
        return;
    }

    if (modem->retry_waiting)
    {
        uint32_t now_ms = ATClient_NowMs(modem->at);

        if ((int32_t)(now_ms - modem->retry_due_ms) < 0)
        {
            return;
        }
        modem->retry_waiting = false;
        modem->command_pending = true;
    }

    if (!modem->command_pending)
    {
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
    if (!modem->started)
    {
        Air780E_PrepareMqttRequest(modem);
    }

    submit_result = ATClient_Submit(modem->at, &modem->request);
    /* Non-blocking: submission only queues the transaction. UART completion
     * and protocol results arrive later through ATClient_Process(). */
    if (submit_result == ATCLIENT_RESULT_OK)
    {
        modem->command_pending = false;
        modem->command_active = true;
    }
    else if (submit_result != ATCLIENT_RESULT_QUEUE_FULL)
    {
        Air780E_FinishStartup(modem, Air780E_MapAtResult(submit_result));
    }
}

Air780E_State Air780E_GetState(const Air780E *modem)
{
    return modem == NULL ? AIR780E_STATE_ERROR : modem->state;
}

const Air780E_Status *Air780E_GetStatus(const Air780E *modem)
{
    return modem == NULL ? NULL : &modem->status;
}

static bool Air780E_BuildCommand(void *context,
                                 char *destination,
                                 size_t capacity,
                                 size_t *length)
{
    Air780E *modem = (Air780E *)context;
    const char *command;
    size_t command_length;

    if (!modem->started)
    {
        return Air780E_BuildMqttCommand(modem,
                                        destination,
                                        capacity,
                                        length);
    }

    command = Air780E_CommandForStage(modem->stage);

    if (command == NULL)
    {
        return false;
    }
    command_length = strlen(command);
    if (command_length > capacity)
    {
        return false;
    }
    memcpy(destination, command, command_length);
    *length = command_length;
    return true;
}

static void Air780E_HandleLine(void *context,
                               const char *line,
                               size_t length)
{
    Air780E *modem = (Air780E *)context;

    if (length >= 6U && memcmp(line, "+CSQ:", 5U) == 0)
    {
        (void)Air780E_ParseCsq(modem, line, length);
        return;
    }
    if (length >= 7U && memcmp(line, "+CPIN:", 6U) == 0)
    {
        static const char ready[] = "+CPIN: READY";

        modem->status.sim_ready =
            length == sizeof(ready) - 1U &&
            memcmp(line, ready, sizeof(ready) - 1U) == 0;
        modem->stage_value_valid = true;
        return;
    }
    if (length >= 8U && memcmp(line, "+CEREG:", 7U) == 0)
    {
        modem->stage_value_valid = Air780E_ParseCereg(modem, line, length);
        return;
    }
    if (length >= 8U && memcmp(line, "+CGATT:", 7U) == 0)
    {
        modem->stage_value_valid = Air780E_ParseCgatt(modem, line, length);
        return;
    }

    if (modem->stage == AIR780E_STAGE_ATI)
    {
        modem->stage_value_valid = Air780E_CopyField(modem->status.model,
                                                     sizeof(modem->status.model),
                                                     line,
                                                     length);
    }
    else if (modem->stage == AIR780E_STAGE_CGMR)
    {
        modem->stage_value_valid = Air780E_CopyField(
            modem->status.firmware,
            sizeof(modem->status.firmware),
            line,
            length);
    }
    else if (modem->stage == AIR780E_STAGE_CGSN)
    {
        size_t index;

        modem->stage_value_valid = length == 15U;
        for (index = 0U; index < length && modem->stage_value_valid; ++index)
        {
            modem->stage_value_valid = line[index] >= '0' && line[index] <= '9';
        }
        if (modem->stage_value_valid)
        {
            (void)Air780E_CopyField(modem->status.imei,
                                    sizeof(modem->status.imei),
                                    line,
                                    length);
        }
    }
}

static void Air780E_CommandComplete(void *context, ATClient_Result result)
{
    Air780E *modem = (Air780E *)context;

    modem->command_active = false;
    if (!modem->started)
    {
        Air780E_MqttCommandComplete(modem, result);
        return;
    }
    if (result != ATCLIENT_RESULT_OK)
    {
        Air780E_ScheduleRetry(modem, Air780E_MapAtResult(result));
        return;
    }

    if (modem->stage == AIR780E_STAGE_AT)
    {
        modem->status.responsive = true;
    }

    if ((modem->stage == AIR780E_STAGE_ATI ||
         modem->stage == AIR780E_STAGE_CGMR ||
         modem->stage == AIR780E_STAGE_CGSN ||
         modem->stage == AIR780E_STAGE_CPIN ||
         modem->stage == AIR780E_STAGE_CEREG ||
         modem->stage == AIR780E_STAGE_CGATT) &&
        !modem->stage_value_valid)
    {
        ++modem->status.parse_error_count;
        Air780E_FinishStartup(modem, AIR780E_RESULT_PARSE_ERROR);
        return;
    }

    if (modem->stage == AIR780E_STAGE_CPIN && !modem->status.sim_ready)
    {
        Air780E_SetState(modem, AIR780E_STATE_SIM_WAIT);
        Air780E_ScheduleRetry(modem, AIR780E_RESULT_NOT_READY);
        return;
    }
    if (modem->stage == AIR780E_STAGE_CEREG &&
        modem->status.registration != 1U &&
        modem->status.registration != 5U)
    {
        Air780E_SetState(modem, AIR780E_STATE_NETWORK_WAIT);
        Air780E_ScheduleRetry(modem, AIR780E_RESULT_NOT_READY);
        return;
    }
    if (modem->stage == AIR780E_STAGE_CGATT &&
        !modem->status.packet_attached)
    {
        Air780E_SetState(modem, AIR780E_STATE_NETWORK_WAIT);
        Air780E_ScheduleRetry(modem, AIR780E_RESULT_NOT_READY);
        return;
    }

    modem->stage_retries = 0U;
    Air780E_AdvanceStage(modem);
}

static bool Air780E_BuildMqttCommand(Air780E *modem,
                                     char *destination,
                                     size_t capacity,
                                     size_t *length)
{
    int written = -1;

    switch (modem->mqtt_step)
    {
    case AIR780E_MQTT_STEP_MCONFIG:
        written = snprintf(destination,
                           capacity,
                           "AT+MCONFIG=\"%s\",\"%s\",\"%s\"",
                           modem->mqtt_config.client_id,
                           modem->mqtt_config.username,
                           modem->mqtt_config.password);
        break;
    case AIR780E_MQTT_STEP_MSGSET:
        written = snprintf(destination, capacity, "AT+MQTTMSGSET=1");
        break;
    case AIR780E_MQTT_STEP_MIPSTART:
        written = snprintf(destination,
                           capacity,
                           "AT+MIPSTART=\"%s\",%lu",
                           modem->mqtt_config.host,
                           (unsigned long)modem->mqtt_config.port);
        break;
    case AIR780E_MQTT_STEP_MCONNECT:
        written = snprintf(destination,
                           capacity,
                           "AT+MCONNECT=%u,%lu",
                           modem->mqtt_config.clean_session ? 1U : 0U,
                           (unsigned long)modem->mqtt_config.keepalive_s);
        break;
    case AIR780E_MQTT_STEP_MSUB:
        written = snprintf(destination,
                           capacity,
                           "AT+MSUB=\"%s\",%u",
                           modem->mqtt_topic,
                           (unsigned int)modem->mqtt_qos);
        break;
    case AIR780E_MQTT_STEP_MPUBEX:
        written = snprintf(destination,
                           capacity,
                           "AT+MPUBEX=\"%s\",%u,%u,%lu",
                           modem->mqtt_topic,
                           (unsigned int)modem->mqtt_qos,
                           modem->mqtt_retain ? 1U : 0U,
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

    if (written <= 0 || (size_t)written >= capacity)
    {
        return false;
    }
    *length = (size_t)written;
    return true;
}

static void Air780E_PrepareMqttRequest(Air780E *modem)
{
    modem->request.timeout_ms = 30000U;

    switch (modem->mqtt_step)
    {
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
        if (modem->mqtt_qos == 1U)
        {
            modem->request.success_tokens[0] = "OK";
            modem->request.success_tokens[1] = "PUBACK";
            modem->request.success_token_count = 2U;
        }
        else if (modem->mqtt_qos == 2U)
        {
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

static void Air780E_MqttCommandComplete(Air780E *modem,
                                        ATClient_Result result)
{
    if (result != ATCLIENT_RESULT_OK)
    {
        Air780E_FinishMqttOperation(modem, Air780E_MapAtResult(result));
        return;
    }

    switch (modem->mqtt_step)
    {
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

static void Air780E_FinishMqttOperation(Air780E *modem,
                                        Air780E_Result result)
{
    Air780E_MqttOperation operation = modem->mqtt_operation;
    bool should_reconnect =
        result != AIR780E_RESULT_OK &&
        operation == AIR780E_MQTT_OPERATION_CONNECT &&
        modem->mqtt_configured && modem->mqtt_config.auto_reconnect &&
        !modem->mqtt_user_disconnect;

    modem->command_pending = false;
    modem->command_active = false;
    modem->mqtt_operation = AIR780E_MQTT_OPERATION_NONE;
    modem->mqtt_step = AIR780E_MQTT_STEP_IDLE;
    modem->mqtt_topic = NULL;
    modem->mqtt_payload = NULL;
    modem->mqtt_payload_length = 0U;
    modem->status.last_mqtt_operation = operation;
    modem->status.last_result = result;
    if (should_reconnect)
    {
        modem->status.mqtt_online = false;
        Air780E_ScheduleMqttReconnect(modem);
    }
    else if (result != AIR780E_RESULT_OK &&
             operation == AIR780E_MQTT_OPERATION_CONNECT)
    {
        modem->status.mqtt_online = false;
        Air780E_SetState(modem, AIR780E_STATE_ERROR);
    }
    if (modem->callbacks.on_mqtt_operation_complete != NULL)
    {
        modem->callbacks.on_mqtt_operation_complete(modem->callback_context,
                                                     operation,
                                                     result);
    }
}

static const char *Air780E_CommandForStage(Air780E_Stage stage)
{
    static const char *const commands[] = {
        NULL,
        "AT",
        "ATE0",
        "AT+CMEE=2",
        "ATI",
        "AT+CGMR",
        "AT+CGSN",
        "AT+CPIN?",
        "AT+CEREG?",
        "AT+CGATT?",
        NULL,
    };

    return stage <= AIR780E_STAGE_COMPLETE ? commands[stage] : NULL;
}

static void Air780E_AdvanceStage(Air780E *modem)
{
    modem->stage = (Air780E_Stage)(modem->stage + 1);
    if (modem->stage == AIR780E_STAGE_CPIN)
    {
        Air780E_SetState(modem, AIR780E_STATE_SIM_WAIT);
    }
    else if (modem->stage == AIR780E_STAGE_CEREG ||
             modem->stage == AIR780E_STAGE_CGATT)
    {
        Air780E_SetState(modem, AIR780E_STATE_NETWORK_WAIT);
    }

    if (modem->stage == AIR780E_STAGE_COMPLETE)
    {
        Air780E_FinishStartup(modem, AIR780E_RESULT_OK);
    }
    else
    {
        modem->command_pending = true;
    }
}

static void Air780E_SetState(Air780E *modem, Air780E_State state)
{
    if (modem->state == state)
    {
        return;
    }
    modem->state = state;
    if (modem->callbacks.on_state_changed != NULL)
    {
        modem->callbacks.on_state_changed(modem->callback_context, state);
    }
}

static void Air780E_FinishStartup(Air780E *modem, Air780E_Result result)
{
    modem->started = false;
    modem->command_pending = false;
    modem->command_active = false;
    modem->retry_waiting = false;
    modem->status.last_result = result;
    Air780E_SetState(modem,
                     result == AIR780E_RESULT_OK ? AIR780E_STATE_OFFLINE
                                                 : AIR780E_STATE_ERROR);
    if (modem->callbacks.on_startup_complete != NULL)
    {
        modem->callbacks.on_startup_complete(modem->callback_context, result);
    }
}

static void Air780E_ScheduleRetry(Air780E *modem, Air780E_Result result)
{
    modem->status.last_result = result;
    if (modem->stage_retries >= AIR780E_MAX_STAGE_RETRIES)
    {
        Air780E_FinishStartup(modem, result);
        return;
    }

    ++modem->stage_retries;
    ++modem->status.retry_count;
    modem->retry_due_ms = ATClient_NowMs(modem->at) + AIR780E_RETRY_DELAY_MS;
    modem->retry_waiting = true;
}

static Air780E_Result Air780E_MapAtResult(ATClient_Result result)
{
    if (result == ATCLIENT_RESULT_TIMEOUT)
    {
        return AIR780E_RESULT_TIMEOUT;
    }
    if (result == ATCLIENT_RESULT_INVALID_ARGUMENT)
    {
        return AIR780E_RESULT_INVALID_ARGUMENT;
    }
    return result == ATCLIENT_RESULT_OK ? AIR780E_RESULT_OK
                                        : AIR780E_RESULT_AT_ERROR;
}

static bool Air780E_CopyField(char *destination,
                              size_t capacity,
                              const char *source,
                              size_t length)
{
    if (length == 0U || length >= capacity)
    {
        return false;
    }
    memcpy(destination, source, length);
    destination[length] = '\0';
    return true;
}

static bool Air780E_ParseUnsigned(const char **cursor,
                                  const char *end,
                                  unsigned int *value)
{
    unsigned int parsed = 0U;
    bool has_digit = false;

    while (*cursor < end && **cursor == ' ')
    {
        ++*cursor;
    }
    while (*cursor < end && **cursor >= '0' && **cursor <= '9')
    {
        has_digit = true;
        parsed = parsed * 10U + (unsigned int)(**cursor - '0');
        ++*cursor;
    }
    if (!has_digit)
    {
        return false;
    }
    *value = parsed;
    return true;
}

static bool Air780E_OnlySpacesRemain(const char *cursor, const char *end)
{
    while (cursor < end && *cursor == ' ')
    {
        ++cursor;
    }
    return cursor == end;
}

static bool Air780E_MqttFieldValid(const char *value, bool allow_empty)
{
    size_t length = 0U;

    if (value == NULL)
    {
        return false;
    }
    while (value[length] != '\0')
    {
        unsigned char byte = (unsigned char)value[length];

        if (length >= AIR780E_MQTT_FIELD_MAX || byte < 0x20U ||
            byte == 0x7FU || byte == (unsigned char)'\"')
        {
            return false;
        }
        ++length;
    }
    return allow_empty || length > 0U;
}

static void Air780E_SubmitCachedGet(Air780E *modem)
{
    ATClient_Result submit_result;

    if (modem->mqtt_cache_notices == 0U || modem->mqtt_get_queued)
    {
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
    if (submit_result == ATCLIENT_RESULT_OK)
    {
        modem->mqtt_get_queued = true;
        modem->mqtt_cache_notices = 0U;
    }
}

static bool Air780E_BuildMqttGet(void *context,
                                 char *destination,
                                 size_t capacity,
                                 size_t *length)
{
    static const char command[] = "AT+MQTTMSGGET";

    (void)context;
    if (capacity < sizeof(command) - 1U)
    {
        return false;
    }
    memcpy(destination, command, sizeof(command) - 1U);
    *length = sizeof(command) - 1U;
    return true;
}

static void Air780E_MqttGetComplete(void *context, ATClient_Result result)
{
    Air780E *modem = (Air780E *)context;
    Air780E_Result mapped = Air780E_MapAtResult(result);

    modem->mqtt_get_queued = false;
    modem->status.last_mqtt_operation = AIR780E_MQTT_OPERATION_CACHE_GET;
    modem->status.last_result = mapped;
    if (modem->callbacks.on_mqtt_operation_complete != NULL)
    {
        modem->callbacks.on_mqtt_operation_complete(
            modem->callback_context,
            AIR780E_MQTT_OPERATION_CACHE_GET,
            mapped);
    }
}

static void Air780E_CacheUrc(void *context,
                             const char *line,
                             size_t length)
{
    Air780E *modem = (Air780E *)context;

    if (memchr(line, ',', length) != NULL)
    {
        return;
    }
    if (modem->mqtt_cache_notices < 4U)
    {
        ++modem->mqtt_cache_notices;
    }
    else
    {
        ++modem->status.mqtt_cache_overwrite_risk;
    }
}

static ATClient_FrameProbeResult Air780E_MqttFrameProbe(
    void *context,
    const uint8_t *header,
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
    if (header_length <= prefix_length)
    {
        return memcmp(header, prefix, header_length) == 0
                   ? ATCLIENT_FRAME_NEED_MORE
                   : ATCLIENT_FRAME_NO_MATCH;
    }
    if (memcmp(header, prefix, prefix_length) != 0)
    {
        return ATCLIENT_FRAME_NO_MATCH;
    }

    topic_start = prefix_length;
    while (topic_start < header_length && header[topic_start] == (uint8_t)' ')
    {
        ++topic_start;
    }
    for (index = topic_start; index < header_length; ++index)
    {
        if (topic_end == 0U)
        {
            if (header[index] == (uint8_t)',')
            {
                if (index == topic_start || index - topic_start > AIR780E_MQTT_FIELD_MAX)
                {
                    return ATCLIENT_FRAME_ERROR;
                }
                topic_end = index;
                continue;
            }
            if (index - topic_start >= AIR780E_MQTT_FIELD_MAX)
            {
                return ATCLIENT_FRAME_ERROR;
            }
            continue;
        }

        if (header[index] == (uint8_t)',')
        {
            if (!has_length_digit || index + 1U != header_length)
            {
                return ATCLIENT_FRAME_ERROR;
            }
            *payload_length = value;
            return value <= ATCLIENT_FRAME_PAYLOAD_MAX
                       ? ATCLIENT_FRAME_READY
                       : ATCLIENT_FRAME_ERROR;
        }
        if (header[index] < (uint8_t)'0' || header[index] > (uint8_t)'9')
        {
            return ATCLIENT_FRAME_ERROR;
        }
        has_length_digit = true;
        value = value * 10U + (size_t)(header[index] - (uint8_t)'0');
        if (value > ATCLIENT_FRAME_PAYLOAD_MAX)
        {
            return ATCLIENT_FRAME_ERROR;
        }
    }
    return ATCLIENT_FRAME_NEED_MORE;
}

static void Air780E_MqttFrameBegin(void *context,
                                    const uint8_t *header,
                                    size_t header_length,
                                    size_t payload_length)
{
    Air780E *modem = (Air780E *)context;
    size_t start = sizeof("+MSUB:") - 1U;
    size_t end = start;

    while (start < header_length && header[start] == (uint8_t)' ')
    {
        ++start;
    }
    end = start;
    while (end < header_length && header[end] != (uint8_t)',')
    {
        ++end;
    }
    memcpy(modem->mqtt_rx_topic, &header[start], end - start);
    modem->mqtt_rx_topic[end - start] = '\0';
    if (modem->callbacks.on_mqtt_message_begin != NULL)
    {
        modem->callbacks.on_mqtt_message_begin(modem->callback_context,
                                               modem->mqtt_rx_topic,
                                               payload_length);
    }
}

static void Air780E_MqttFrameData(void *context,
                                  const uint8_t *data,
                                  size_t length)
{
    Air780E *modem = (Air780E *)context;

    if (modem->callbacks.on_mqtt_message_data != NULL)
    {
        modem->callbacks.on_mqtt_message_data(modem->callback_context,
                                              data,
                                              length);
    }
}

static void Air780E_MqttFrameEnd(void *context, ATClient_Result result)
{
    Air780E *modem = (Air780E *)context;
    Air780E_Result mapped = Air780E_MapAtResult(result);

    if (result == ATCLIENT_RESULT_OK)
    {
        ++modem->status.mqtt_message_count;
    }
    if (modem->callbacks.on_mqtt_message_end != NULL)
    {
        modem->callbacks.on_mqtt_message_end(modem->callback_context, mapped);
    }
}

static void Air780E_ClosedUrc(void *context,
                              const char *line,
                              size_t length)
{
    Air780E *modem = (Air780E *)context;

    (void)line;
    (void)length;
    modem->status.mqtt_online = false;
    if (modem->mqtt_user_disconnect)
    {
        Air780E_SetState(modem, AIR780E_STATE_OFFLINE);
        return;
    }
    Air780E_ScheduleMqttReconnect(modem);
}

static void Air780E_MqttStatusUrc(void *context,
                                  const char *line,
                                  size_t length)
{
    Air780E *modem = (Air780E *)context;
    const size_t prefix_length = sizeof("+MQTTSTATU:") - 1U;
    const char *cursor = line + prefix_length;
    const char *end = line + length;
    unsigned int value;

    if (length <= prefix_length ||
        !Air780E_ParseUnsigned(&cursor, end, &value) ||
        !Air780E_OnlySpacesRemain(cursor, end) || value > 2U)
    {
        ++modem->status.parse_error_count;
        return;
    }

    if (value == 0U)
    {
        modem->status.mqtt_online = false;
        if (!modem->mqtt_user_disconnect)
        {
            Air780E_ScheduleMqttReconnect(modem);
        }
    }
    else if (value == 1U)
    {
        modem->status.mqtt_online = false;
        Air780E_SetState(modem, AIR780E_STATE_MQTT_AUTHENTICATING);
    }
    else
    {
        modem->status.mqtt_online = true;
        modem->mqtt_reconnect_index = 0U;
        modem->mqtt_reconnect_waiting = false;
        Air780E_SetState(modem, AIR780E_STATE_MQTT_ONLINE);
    }
}

static void Air780E_ScheduleMqttReconnect(Air780E *modem)
{
    static const uint32_t delays_ms[] = {
        1000U, 2000U, 4000U, 8000U, 16000U, 30000U,
    };
    uint8_t index;

    if (!modem->mqtt_configured || !modem->mqtt_config.auto_reconnect ||
        modem->mqtt_user_disconnect)
    {
        Air780E_SetState(modem, AIR780E_STATE_OFFLINE);
        return;
    }
    if (modem->mqtt_reconnect_waiting)
    {
        return;
    }

    index = modem->mqtt_reconnect_index;
    if (index >= sizeof(delays_ms) / sizeof(delays_ms[0]))
    {
        index = (uint8_t)(sizeof(delays_ms) / sizeof(delays_ms[0]) - 1U);
    }
    modem->mqtt_reconnect_due_ms = ATClient_NowMs(modem->at) + delays_ms[index];
    if (modem->mqtt_reconnect_index <
        sizeof(delays_ms) / sizeof(delays_ms[0]) - 1U)
    {
        ++modem->mqtt_reconnect_index;
    }
    modem->mqtt_reconnect_waiting = true;
    Air780E_SetState(modem, AIR780E_STATE_RECONNECT_WAIT);
}

static bool Air780E_ParseCereg(Air780E *modem,
                               const char *line,
                               size_t length)
{
    const char *cursor = line + 7U;
    const char *end = line + length;
    unsigned int first;
    unsigned int second;

    if (!Air780E_ParseUnsigned(&cursor, end, &first))
    {
        return false;
    }
    while (cursor < end && *cursor == ' ')
    {
        ++cursor;
    }
    if (cursor < end && *cursor == ',')
    {
        ++cursor;
        if (!Air780E_ParseUnsigned(&cursor, end, &second))
        {
            return false;
        }
        first = second;
    }
    if (!Air780E_OnlySpacesRemain(cursor, end) || first > 5U)
    {
        return false;
    }
    modem->status.registration = (uint8_t)first;
    return true;
}

static bool Air780E_ParseCgatt(Air780E *modem,
                               const char *line,
                               size_t length)
{
    const char *cursor = line + 7U;
    const char *end = line + length;
    unsigned int value;

    if (!Air780E_ParseUnsigned(&cursor, end, &value) ||
        !Air780E_OnlySpacesRemain(cursor, end) || value > 1U)
    {
        return false;
    }
    modem->status.packet_attached = value == 1U;
    return true;
}

static bool Air780E_ParseCsq(Air780E *modem,
                             const char *line,
                             size_t length)
{
    const char *cursor = line + 5U;
    const char *end = line + length;
    unsigned int rssi;
    unsigned int ber;

    if (!Air780E_ParseUnsigned(&cursor, end, &rssi) || cursor >= end ||
        *cursor != ',')
    {
        return false;
    }
    ++cursor;
    if (!Air780E_ParseUnsigned(&cursor, end, &ber) ||
        !Air780E_OnlySpacesRemain(cursor, end) ||
        (rssi > 31U && rssi != 99U) || (ber > 7U && ber != 99U))
    {
        return false;
    }

    modem->status.rssi = (uint8_t)rssi;
    modem->status.ber = (uint8_t)ber;
    modem->status.rssi_known = rssi != 99U;
    modem->status.rssi_dbm = modem->status.rssi_known
                                 ? (int16_t)((int)rssi * 2 - 113)
                                 : 0;
    return true;
}
