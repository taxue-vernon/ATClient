#ifndef ATCLIENT_H
#define ATCLIENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ATCLIENT_RX_CAPACITY       1024U
#define ATCLIENT_LINE_CAPACITY     384U
#define ATCLIENT_COMMAND_CAPACITY  1024U
#define ATCLIENT_QUEUE_CAPACITY    4U
#define ATCLIENT_URC_CAPACITY      8U
#define ATCLIENT_SUCCESS_TOKEN_CAPACITY  3U
#define ATCLIENT_FAILURE_TOKEN_CAPACITY  3U
#define ATCLIENT_LENGTH_FRAME_CAPACITY  4U
#define ATCLIENT_FRAME_PAYLOAD_MAX      4100U

typedef enum
{
    ATCLIENT_RESULT_OK = 0,
    ATCLIENT_RESULT_BUSY,
    ATCLIENT_RESULT_QUEUE_FULL,
    ATCLIENT_RESULT_INVALID_ARGUMENT,
    ATCLIENT_RESULT_TRANSPORT_ERROR,
    ATCLIENT_RESULT_ERROR_RESPONSE,
    ATCLIENT_RESULT_CME_ERROR,
    ATCLIENT_RESULT_CMS_ERROR,
    ATCLIENT_RESULT_TIMEOUT,
    ATCLIENT_RESULT_RX_OVERFLOW,
    ATCLIENT_RESULT_LINE_TOO_LONG,
    ATCLIENT_RESULT_PROTOCOL_ERROR,
    ATCLIENT_RESULT_CANCELLED
} ATClient_Result;

typedef enum
{
    ATCLIENT_PRIORITY_NORMAL = 0,
    ATCLIENT_PRIORITY_URGENT
} ATClient_Priority;

typedef bool (*ATClient_StartTx)(void *context,
                                 const uint8_t *data,
                                 size_t length);
typedef uint32_t (*ATClient_GetTimeMs)(void *context);

typedef struct
{
    ATClient_StartTx start_tx;
    ATClient_GetTimeMs get_time_ms;
    void *context;
} ATClient_Transport;

typedef bool (*ATClient_BuildCommand)(void *context,
                                      char *destination,
                                      size_t capacity,
                                      size_t *length);
typedef void (*ATClient_LineCallback)(void *context,
                                      const char *line,
                                      size_t length);
typedef void (*ATClient_CompleteCallback)(void *context,
                                          ATClient_Result result);
typedef void (*ATClient_UrcCallback)(void *context,
                                     const char *line,
                                     size_t length);

typedef struct
{
    ATClient_BuildCommand build_command;
    void *command_context;
    uint8_t prompt_byte;
    const uint8_t *payload;
    size_t payload_length;
    const char *success_tokens[ATCLIENT_SUCCESS_TOKEN_CAPACITY];
    size_t success_token_count;
    const char *failure_tokens[ATCLIENT_FAILURE_TOKEN_CAPACITY];
    size_t failure_token_count;
    uint32_t timeout_ms;
    ATClient_Priority priority;
    ATClient_LineCallback on_line;
    ATClient_CompleteCallback on_complete;
    void *callback_context;
} ATClient_Request;

typedef struct
{
    uint32_t rx_overflow_count;
    uint32_t rx_dropped_bytes;
    uint32_t line_too_long_count;
    uint32_t queue_full_count;
    uint32_t transport_error_count;
    uint32_t protocol_error_count;
} ATClient_Stats;

typedef enum
{
    ATCLIENT_FRAME_NO_MATCH = 0,
    ATCLIENT_FRAME_NEED_MORE,
    ATCLIENT_FRAME_READY,
    ATCLIENT_FRAME_ERROR
} ATClient_FrameProbeResult;

typedef ATClient_FrameProbeResult (*ATClient_LengthFrameProbe)(
    void *context,
    const uint8_t *header,
    size_t header_length,
    size_t *payload_length);
typedef void (*ATClient_LengthFrameBegin)(void *context,
                                          const uint8_t *header,
                                          size_t header_length,
                                          size_t payload_length);
typedef void (*ATClient_LengthFrameData)(void *context,
                                         const uint8_t *data,
                                         size_t length);
typedef void (*ATClient_LengthFrameEnd)(void *context,
                                        ATClient_Result result);

typedef struct
{
    const char *prefix;
    ATClient_LengthFrameProbe probe;
    ATClient_LengthFrameBegin on_begin;
    ATClient_LengthFrameData on_data;
    ATClient_LengthFrameEnd on_end;
    void *context;
} ATClient_LengthFrameRegistration;

typedef struct
{
    ATClient_LengthFrameRegistration registration;
    size_t prefix_length;
} ATClient_LengthFrameSlot;

typedef struct
{
    const char *prefix;
    size_t prefix_length;
    ATClient_UrcCallback callback;
    void *context;
} ATClient_UrcRegistration;

typedef enum
{
    ATCLIENT_STATE_IDLE = 0,
    ATCLIENT_STATE_TX_ACTIVE,
    ATCLIENT_STATE_WAIT_PROMPT,
    ATCLIENT_STATE_PAYLOAD_TX_ACTIVE,
    ATCLIENT_STATE_WAIT_RESPONSE
} ATClient_State;

typedef enum
{
    ATCLIENT_PARSER_LINE = 0,
    ATCLIENT_PARSER_FRAME_PAYLOAD
} ATClient_ParserState;

/* The object uses fixed storage only. Request callback contexts must remain
 * valid from ATClient_Submit() until the completion callback runs. */
typedef struct
{
    ATClient_Transport transport;
    ATClient_Request queue[ATCLIENT_QUEUE_CAPACITY];
    size_t queue_count;
    ATClient_Request active_request;
    bool has_active_request;
    volatile ATClient_State state;
    uint32_t started_ms;
    volatile bool transport_error_pending;
    bool prompt_seen;
    bool skip_prompt_space;
    size_t success_token_index;

    ATClient_UrcRegistration urcs[ATCLIENT_URC_CAPACITY];
    size_t urc_count;
    ATClient_LengthFrameSlot length_frames[ATCLIENT_LENGTH_FRAME_CAPACITY];
    size_t length_frame_count;
    ATClient_ParserState parser_state;
    size_t active_length_frame;
    size_t frame_payload_remaining;

    uint8_t rx_buffer[ATCLIENT_RX_CAPACITY];
    /* Single-producer/single-consumer monotonic counters: ISR writes head,
     * main-loop parser writes tail. No shared read-modify-write count. */
    volatile uint32_t rx_head;
    volatile uint32_t rx_tail;

    char line_buffer[ATCLIENT_LINE_CAPACITY];
    size_t line_length;
    bool discarding_line;

    uint8_t command_buffer[ATCLIENT_COMMAND_CAPACITY + 2U];
    size_t command_length;

    ATClient_Stats stats;
} ATClient;

bool ATClient_Init(ATClient *client, const ATClient_Transport *transport);
ATClient_Result ATClient_Submit(ATClient *client,
                                const ATClient_Request *request);
ATClient_Result ATClient_PushRx(ATClient *client,
                                const uint8_t *data,
                                size_t length);
ATClient_Result ATClient_RegisterUrc(ATClient *client,
                                     const char *prefix,
                                     ATClient_UrcCallback callback,
                                     void *context);
ATClient_Result ATClient_RegisterLengthFrame(
    ATClient *client,
    const ATClient_LengthFrameRegistration *registration);
void ATClient_OnTxComplete(ATClient *client);
void ATClient_OnTransportError(ATClient *client);
void ATClient_Process(ATClient *client);
bool ATClient_IsIdle(const ATClient *client);
const ATClient_Stats *ATClient_GetStats(const ATClient *client);
uint32_t ATClient_NowMs(const ATClient *client);
size_t ATClient_RxPending(const ATClient *client);

#ifdef __cplusplus
}
#endif

#endif
