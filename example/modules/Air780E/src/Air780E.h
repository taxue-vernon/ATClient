#ifndef AIR780E_H
#define AIR780E_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "ATClient.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AIR780E_MODEL_CAPACITY     32U
#define AIR780E_FIRMWARE_CAPACITY  64U
#define AIR780E_IMEI_CAPACITY      16U
#define AIR780E_MQTT_FIELD_MAX     256U

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

typedef enum
{
    AIR780E_MQTT_OPERATION_NONE = 0,
    AIR780E_MQTT_OPERATION_CONNECT,
    AIR780E_MQTT_OPERATION_SUBSCRIBE,
    AIR780E_MQTT_OPERATION_PUBLISH,
    AIR780E_MQTT_OPERATION_DISCONNECT,
    AIR780E_MQTT_OPERATION_CACHE_GET
} Air780E_MqttOperation;

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

typedef struct
{
    void (*on_state_changed)(void *context, Air780E_State state);
    void (*on_startup_complete)(void *context, Air780E_Result result);
    void (*on_mqtt_operation_complete)(void *context,
                                       Air780E_MqttOperation operation,
                                       Air780E_Result result);
    void (*on_mqtt_message_begin)(void *context,
                                  const char *topic,
                                  size_t payload_length);
    void (*on_mqtt_message_data)(void *context,
                                 const uint8_t *data,
                                 size_t length);
    void (*on_mqtt_message_end)(void *context, Air780E_Result result);
} Air780E_Callbacks;

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

bool Air780E_Init(Air780E *modem,
                  ATClient *at,
                  const Air780E_Callbacks *callbacks,
                  void *callback_context);
Air780E_Result Air780E_Start(Air780E *modem);
/* Configuration strings are borrowed and must remain valid for the lifetime
 * of the Air780E handle or until the next successful configuration call. */
Air780E_Result Air780E_MqttConfigure(Air780E *modem,
                                     const Air780E_MqttConfig *config);
Air780E_Result Air780E_MqttConnect(Air780E *modem);
/* Topic storage is borrowed until the completion callback for this operation. */
Air780E_Result Air780E_MqttSubscribe(Air780E *modem,
                                     const char *topic,
                                     uint8_t qos);
/* Topic and payload are borrowed until the publish completion callback. */
Air780E_Result Air780E_MqttPublish(Air780E *modem,
                                   const char *topic,
                                   const uint8_t *payload,
                                   size_t length,
                                   uint8_t qos,
                                   bool retain);
Air780E_Result Air780E_MqttDisconnect(Air780E *modem);
void Air780E_Process(Air780E *modem);
Air780E_State Air780E_GetState(const Air780E *modem);
const Air780E_Status *Air780E_GetStatus(const Air780E *modem);

#ifdef __cplusplus
}
#endif

#endif
