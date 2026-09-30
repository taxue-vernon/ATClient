#ifndef ML307C_H
#define ML307C_H
#include "ATClient.h"
#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ML307C_RESULT_OK, ML307C_RESULT_BUSY, ML307C_RESULT_INVALID_ARGUMENT,
    ML307C_RESULT_AT_ERROR, ML307C_RESULT_TIMEOUT, ML307C_RESULT_PARSE_ERROR,
    ML307C_RESULT_NOT_READY, ML307C_RESULT_NETWORK_LOST, ML307C_RESULT_HTTP_ERROR
} ML307C_Result;
typedef enum {
    ML307C_STATE_OFFLINE, ML307C_STATE_PROBING, ML307C_STATE_SIM_WAIT,
    ML307C_STATE_NETWORK_WAIT, ML307C_STATE_DATA_ACTIVATING,
    ML307C_STATE_NETWORK_READY, ML307C_STATE_RETRY_WAIT, ML307C_STATE_ERROR
} ML307C_State;
typedef enum { ML307C_HTTP_HEADER, ML307C_HTTP_CONTENT } ML307C_HttpDataType;
typedef struct {
    const char *host;              /* http(s)://authority[:port], borrowed */
    uint16_t read_chunk_size;      /* 0 = 256; 1..1024 */
    const uint8_t *ca_pem;         /* HTTPS PEM, borrowed until prepared */
    size_t ca_length;
    const char *ca_name;           /* versioned modem certificate name */
} ML307C_HttpConfig;
typedef struct {
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
typedef struct {
    void (*on_state_changed)(void *, ML307C_State);
    void (*on_startup_complete)(void *, ML307C_Result);
    void (*on_http_response)(void *, uint16_t, size_t, size_t);
    void (*on_http_data)(void *, ML307C_HttpDataType, const uint8_t *, size_t);
    void (*on_http_complete)(void *, ML307C_Result);
    void (*on_sms_complete)(void *, ML307C_Result, uint32_t message_reference);
} ML307C_Callbacks;

/* Internal cooperative stages; applications use APIs instead of editing these. */
typedef enum {
    ML_N_IDLE, ML_N_AT, ML_N_ECHO, ML_N_CMEE, ML_N_MODEL, ML_N_FW,
    ML_N_IMEI, ML_N_SIM, ML_N_CFUN, ML_N_CSQ, ML_N_REG, ML_N_PDP,
    ML_N_ACTIVATE, ML_N_WAIT,
    ML_T_CLOCK, ML_T_LIST, ML_T_WRITE, ML_T_CERT, ML_T_AUTH, ML_T_STAMP,
    ML_T_VERIFY, ML_T_SNI,
    ML_H_CREATE, ML_H_SSL, ML_H_CACHE, ML_H_ENCODING, ML_H_TIMEOUT, ML_H_HEADER, ML_H_BODY,
    ML_H_REQUEST, ML_H_WAIT, ML_H_READ, ML_H_DELETE,
    ML_S_MODE, ML_S_SEND, ML_P_SLEEP
} ML307C_Stage;
typedef struct {
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

bool ML307C_Init(ML307C *, ATClient *, const ML307C_Callbacks *, void *);
ML307C_Result ML307C_Start(ML307C *);
/* Non-blocking: call repeatedly next to ATClient_Process(). All waits are
 * represented by state/deadlines; callbacks execute in the main context. */
void ML307C_Process(ML307C *);
/* Configure temporary ML307C sleep mode while idle. Only raise DTR after
 * SleepReady returns true; EnterSleep then suppresses all AT polling.
 * Lower DTR before Wake; Wake rechecks network and HTTPS state. */
void ML307C_RequestIdleSleep(ML307C *, bool enabled);
bool ML307C_SleepReady(const ML307C *);
bool ML307C_EnterSleep(ML307C *);
void ML307C_Wake(ML307C *);
ML307C_State ML307C_GetState(const ML307C *);
const ML307C_Status *ML307C_GetStatus(const ML307C *);
/* Config host remains valid until reconfiguration. path/body remain valid
 * until on_http_complete. Callbacks must return promptly and not block. */
bool ML307C_HttpConfigure(ML307C *, const ML307C_HttpConfig *);
ML307C_Result ML307C_HttpGet(ML307C *, const char *);
ML307C_Result ML307C_HttpPost(ML307C *, const char *, const char *,
                            const uint8_t *, size_t);
/* Non-blocking, one SMS at a time. Printable ASCII text, 1..160 bytes.
 * No automatic retry: a timeout may occur after the network accepted it. */
ML307C_Result ML307C_SendSms(ML307C *, const char *number, const char *text);
#ifdef __cplusplus
}
#endif
#endif
