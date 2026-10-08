/**
 * @file    ATClient.h
 * @brief   通用非阻塞 AT 指令客户端：请求队列、响应匹配、URC 分发与定长二进制帧解析。
 * @details 只使用固定大小的静态存储，不调用 malloc。典型用法：
 *          1. 填写 ATClient_Transport（发送函数 + 毫秒时钟），调用 ATClient_Init()；
 *          2. 串口接收中断/DMA 回调中调用 ATClient_PushRx()，发送完成调用 ATClient_OnTxComplete()；
 *          3. 主循环反复调用 ATClient_Process()；
 *          4. 通过 ATClient_Submit() 提交请求，在 on_complete 回调里获取结果。
 */

#ifndef ATCLIENT_H
#define ATCLIENT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ATCLIENT_RX_CAPACITY            1024U
#define ATCLIENT_LINE_CAPACITY          384U
#define ATCLIENT_COMMAND_CAPACITY       1024U
#define ATCLIENT_QUEUE_CAPACITY         4U
#define ATCLIENT_URC_CAPACITY           8U
#define ATCLIENT_SUCCESS_TOKEN_CAPACITY 3U
#define ATCLIENT_FAILURE_TOKEN_CAPACITY 3U
#define ATCLIENT_LENGTH_FRAME_CAPACITY  4U
#define ATCLIENT_FRAME_PAYLOAD_MAX      4100U

/** @brief AT 请求的完成结果码。 */
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

/** @brief 请求优先级：URGENT 插入到队列中所有 URGENT 请求之后、NORMAL 请求之前。 */
typedef enum
{
    ATCLIENT_PRIORITY_NORMAL = 0,
    ATCLIENT_PRIORITY_URGENT
} ATClient_Priority;

/**
 * @brief 传输层“启动发送”回调：应立即启动异步发送（如 DMA）并返回。
 * @param context 由 ATClient 传入：ATClient_Transport::context 中保存的指针（如端口状态）。
 * @param data    由 ATClient 传入：要发送的字节（指向客户端内部缓冲或请求的 payload），
 *                发送完成前保持有效。
 * @param length  由 ATClient 传入：要发送的字节数。
 * @return true 已成功启动发送；false 启动失败（请求以 TRANSPORT_ERROR 结束）。
 */
typedef bool (*ATClient_StartTx)(void *context, const uint8_t *data, size_t length);

/**
 * @brief 传输层毫秒时钟回调，返回单调递增（允许 32 位回绕）的毫秒计数。
 * @param context 由 ATClient 传入：ATClient_Transport::context 中保存的指针。
 * @return 当前毫秒时间，例如 HAL_GetTick()。
 */
typedef uint32_t (*ATClient_GetTimeMs)(void *context);

/**
 * @brief 底层传输接口，ATClient_Init() 会复制一份。
 * @note  start_tx 与 get_time_ms 必须非 NULL；context 原样传给两个回调。
 */
typedef struct
{
    ATClient_StartTx start_tx;
    ATClient_GetTimeMs get_time_ms;
    void *context;
} ATClient_Transport;

/**
 * @brief 构造命令文本的回调（不含结尾 "\r\n"，由客户端追加）。
 * @param context     由 ATClient 传入：ATClient_Request::command_context 中保存的指针。
 * @param destination 由 ATClient 传入：命令写入的缓冲区，在这里写命令文本（不要写 "\r
 *                    "）。
 * @param capacity    由 ATClient 传入：destination 的容量（ATCLIENT_COMMAND_CAPACITY）。
 * @param length      输出：把实际写入的字节数写到 *length，取值 1..capacity。
 * @return true 构造成功；false 构造失败（请求以 INVALID_ARGUMENT 结束）。
 */
typedef bool (*ATClient_BuildCommand)(void *context, char *destination, size_t capacity,
                                      size_t *length);

/**
 * @brief 收到属于当前请求的普通响应行（非 OK/ERROR/URC）时调用。
 * @param context 由 ATClient 传入：ATClient_Request::callback_context 中保存的指针。
 * @param line    由 ATClient 传入：一行响应文本，以 '\0' 结尾、不含 "\r
 *                "，
 *                只在回调期间有效，需要保存请复制。
 * @param length  由 ATClient 传入：line 的字节数。
 */
typedef void (*ATClient_LineCallback)(void *context, const char *line, size_t length);

/**
 * @brief 请求结束（成功、失败、超时、取消）时调用，每个请求恰好调用一次。
 * @param context 由 ATClient 传入：ATClient_Request::callback_context 中保存的指针。
 * @param result  由 ATClient 传入：请求的完成结果，ATCLIENT_RESULT_OK 表示成功。
 */
typedef void (*ATClient_CompleteCallback)(void *context, ATClient_Result result);

/**
 * @brief URC（主动上报）行回调。
 * @param context 由 ATClient 传入：ATClient_RegisterUrc() 时给的 context。
 * @param line    由 ATClient 传入：完整的 URC 行，以 '\0' 结尾，只在回调期间有效。
 * @param length  由 ATClient 传入：line 的字节数。
 */
typedef void (*ATClient_UrcCallback)(void *context, const char *line, size_t length);

/**
 * @brief 一条 AT 请求的描述，ATClient_Submit() 时按值复制进队列。
 * @details 主要字段：
 *          - build_command / command_context：命令构造回调及其上下文；
 *          - prompt_byte：非 0 时等待该提示符（如 '>'）后再发送 payload；
 *          - payload / payload_length：提示符后发送的数据，需保持有效直到完成回调；
 *          - success_tokens：按顺序匹配的成功行；为 0 个时以 "OK" 作为成功；
 *          - failure_tokens：命中任意一个即判为 ERROR_RESPONSE；
 *          - timeout_ms：从开始发送算起的超时时间，必须大于 0；
 *          - on_line / on_complete / callback_context：响应行与完成回调。
 */
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

/** @brief 运行期错误统计计数，可用 ATClient_GetStats() 读取。 */
typedef struct
{
    uint32_t rx_overflow_count;
    uint32_t rx_dropped_bytes;
    uint32_t line_too_long_count;
    uint32_t queue_full_count;
    uint32_t transport_error_count;
    uint32_t protocol_error_count;
} ATClient_Stats;

/** @brief 定长帧探测结果：不匹配 / 需要更多字节 / 帧头完整 / 帧头格式错误。 */
typedef enum
{
    ATCLIENT_FRAME_NO_MATCH = 0,
    ATCLIENT_FRAME_NEED_MORE,
    ATCLIENT_FRAME_READY,
    ATCLIENT_FRAME_ERROR
} ATClient_FrameProbeResult;

/**
 * @brief 定长帧头探测回调，每收到一个字节都会以当前行内容调用一次。
 * @param context        由 ATClient 传入：注册时 registration.context 的值。
 * @param header         由 ATClient 传入：当前这行已经收到的字节（还没遇到 "\r
 *                       "）。
 * @param header_length  由 ATClient 传入：header 的字节数。
 * @param payload_length 输出：返回 READY 时，把后面二进制负载的字节数写到这里。
 * @return 见 ATClient_FrameProbeResult。
 */
typedef ATClient_FrameProbeResult (*ATClient_LengthFrameProbe)(void *context, const uint8_t *header,
                                                               size_t header_length,
                                                               size_t *payload_length);

/**
 * @brief 定长帧开始回调，帧头识别完成后调用一次。
 * @param context        由 ATClient 传入：注册时 registration.context 的值。
 * @param header         由 ATClient 传入：完整的帧头字节。
 * @param header_length  由 ATClient 传入：header 的字节数。
 * @param payload_length 由 ATClient 传入：随后将收到的负载字节数。
 */
typedef void (*ATClient_LengthFrameBegin)(void *context, const uint8_t *header,
                                          size_t header_length, size_t payload_length);

/**
 * @brief 定长帧负载数据回调，可能分多次调用。
 * @param context 由 ATClient 传入：注册时 registration.context 的值。
 * @param data    由 ATClient 传入：本次收到的负载片段，只在回调期间有效。
 * @param length  由 ATClient 传入：data 的字节数。
 */
typedef void (*ATClient_LengthFrameData)(void *context, const uint8_t *data, size_t length);

/**
 * @brief 定长帧结束回调：OK 表示收齐；其他值表示因请求失败被中止。
 * @param context 由 ATClient 传入：注册时 registration.context 的值。
 * @param result  由 ATClient 传入：ATCLIENT_RESULT_OK 表示收齐，其他值表示请求失败而中止。
 */
typedef void (*ATClient_LengthFrameEnd)(void *context, ATClient_Result result);

/** @brief 定长二进制帧（如 "+MSUB: topic,len,<bytes>"）的注册信息。 */
typedef struct
{
    const char *prefix;
    ATClient_LengthFrameProbe probe;
    ATClient_LengthFrameBegin on_begin;
    ATClient_LengthFrameData on_data;
    ATClient_LengthFrameEnd on_end;
    void *context;
} ATClient_LengthFrameRegistration;

/** @brief 客户端内部保存的定长帧注册槽（附带缓存的前缀长度）。 */
typedef struct
{
    ATClient_LengthFrameRegistration registration;
    size_t prefix_length;
} ATClient_LengthFrameSlot;

/** @brief 客户端内部保存的 URC 前缀注册项。 */
typedef struct
{
    const char *prefix;
    size_t prefix_length;
    ATClient_UrcCallback callback;
    void *context;
} ATClient_UrcRegistration;

/** @brief 当前活动请求的收发状态机。 */
typedef enum
{
    ATCLIENT_STATE_IDLE = 0,
    ATCLIENT_STATE_TX_ACTIVE,
    ATCLIENT_STATE_WAIT_PROMPT,
    ATCLIENT_STATE_PAYLOAD_TX_ACTIVE,
    ATCLIENT_STATE_WAIT_RESPONSE
} ATClient_State;

/** @brief 接收解析器状态：按行解析，或正在接收定长帧负载。 */
typedef enum
{
    ATCLIENT_PARSER_LINE = 0,
    ATCLIENT_PARSER_FRAME_PAYLOAD
} ATClient_ParserState;

/* The object uses fixed storage only. Request callback contexts must remain
 * valid from ATClient_Submit() until the completion callback runs. */
/** @brief AT 客户端对象，全部存储为定长数组，建议定义为静态变量。 */
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

/**
 * @brief 初始化 AT 客户端并绑定底层传输接口。
 * @details 清零整个对象（队列、URC 表、接收缓冲、统计），再复制 transport。
 * @param client    传要初始化的 ATClient 变量地址（如 &at，建议定义为 static），不能为 NULL。
 * @param transport 传 ATClient_Transport 结构体地址，通常是 ML307C_UartPort_GetTransport()
 *                  的返回值；
 *                  start_tx 与 get_time_ms 不能为 NULL，内容会被复制，调用后可以释放。
 * @return true 初始化成功；false 参数无效。
 * @par 示例
 * @code
 * static ATClient at;
 * ATClient_Transport transport = ML307C_UartPort_GetTransport();
 * if (!ATClient_Init(&at, &transport)) {
 *     Error_Handler();
 * }
 * @endcode
 */
bool ATClient_Init(ATClient *client, const ATClient_Transport *transport);

/**
 * @brief 把一条请求加入发送队列（非阻塞）。
 * @details 请求按值复制；URGENT 请求排在已有 URGENT 之后、NORMAL 之前。
 *          真正的发送在下一次 ATClient_Process() 中开始。
 * @param client  传已用 ATClient_Init() 初始化的 ATClient 地址（如 &at）。
 * @param request 传填好的 ATClient_Request 地址；至少要设置 build_command 和 timeout_ms，
 *                内容被复制，调用后可以释放（payload 与回调上下文除外）。
 * @return ATCLIENT_RESULT_OK 已入队；
 *         ATCLIENT_RESULT_QUEUE_FULL 队列已满；
 *         ATCLIENT_RESULT_INVALID_ARGUMENT 参数或字段组合无效。
 * @par 示例
 * @code
 * static bool build_at(void *ctx, char *dst, size_t cap, size_t *len)
 * {
 *     (void)ctx;
 *     *len = (size_t)snprintf(dst, cap, "AT+CSQ");
 *     return *len > 0U && *len < cap;
 * }
 *
 * ATClient_Request req = {0};
 * req.build_command = build_at;
 * req.timeout_ms = 1000U;
 * req.on_line = on_csq_line;       // 处理 "+CSQ: 20,99"
 * req.on_complete = on_csq_done;   // 收到 OK/ERROR/超时后调用
 * (void)ATClient_Submit(&at, &req);
 * @endcode
 */
ATClient_Result ATClient_Submit(ATClient *client, const ATClient_Request *request);

/**
 * @brief 把串口收到的原始字节写入接收环形缓冲区（可在中断中调用）。
 * @param client 传已用 ATClient_Init() 初始化的 ATClient 地址（如 &at）。
 * @param data   传串口收到的数据（如 DMA 接收缓冲区中新到的那一段）；length 为 0 时可传 NULL。
 * @param length 传 data 中的字节数。
 * @return ATCLIENT_RESULT_OK 全部写入；
 *         ATCLIENT_RESULT_RX_OVERFLOW 缓冲区满，多余字节被丢弃并计入统计；
 *         ATCLIENT_RESULT_INVALID_ARGUMENT 参数无效。
 * @par 示例
 * @code
 * void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
 * {
 *     (void)ATClient_PushRx(&at, dma_rx_buffer, size);
 * }
 * @endcode
 */
ATClient_Result ATClient_PushRx(ATClient *client, const uint8_t *data, size_t length);

/**
 * @brief 注册（或更新）一个 URC 行前缀回调。
 * @details 前缀相同则覆盖原回调；匹配到的行不再交给当前请求的 on_line。
 * @param client   传已用 ATClient_Init() 初始化的 ATClient 地址（如 &at）。
 * @param prefix   传 URC 行的前缀字符串，如 "+CEREG:"；只保存指针，请传字符串常量。
 * @param callback 传收到该 URC 时要调用的函数，不能为 NULL。
 * @param context  传任意指针，回调时原样交给 callback 的 context（不需要可传 NULL）。
 * @return ATCLIENT_RESULT_OK 注册成功；
 *         ATCLIENT_RESULT_QUEUE_FULL 注册表已满（ATCLIENT_URC_CAPACITY）；
 *         ATCLIENT_RESULT_INVALID_ARGUMENT 参数无效或前缀为空/过长。
 * @par 示例
 * @code
 * static void on_cereg(void *ctx, const char *line, size_t length)
 * {
 *     (void)ctx;
 *     printf("URC: %.*s\r\n", (int)length, line);
 * }
 *
 * (void)ATClient_RegisterUrc(&at, "+CEREG:", on_cereg, NULL);
 * @endcode
 */
ATClient_Result ATClient_RegisterUrc(ATClient *client, const char *prefix,
                                     ATClient_UrcCallback callback, void *context);

/**
 * @brief 注册（或更新）一种“帧头 + 定长二进制负载”的接收格式。
 * @details 适用于 "+MSUB: <topic>,<len>,<bytes>" 这类负载中可能含 "\r\n" 的上报。
 * @param client       传已用 ATClient_Init() 初始化的 ATClient 地址（如 &at）。
 * @param registration 传填好的注册结构体地址；prefix、probe、on_begin、on_data、on_end
 *                     都不能为 NULL，prefix 只保存指针，请传字符串常量。
 * @return ATCLIENT_RESULT_OK 注册成功；
 *         ATCLIENT_RESULT_QUEUE_FULL 注册表已满（ATCLIENT_LENGTH_FRAME_CAPACITY）；
 *         ATCLIENT_RESULT_INVALID_ARGUMENT 参数无效。
 * @par 示例
 * @code
 * ATClient_LengthFrameRegistration reg = {
 *     .prefix = "+MSUB:",
 *     .probe = msub_probe,
 *     .on_begin = msub_begin,
 *     .on_data = msub_data,
 *     .on_end = msub_end,
 *     .context = &modem,
 * };
 * (void)ATClient_RegisterLengthFrame(&at, &reg);
 * @endcode
 */
ATClient_Result ATClient_RegisterLengthFrame(ATClient *client,
                                             const ATClient_LengthFrameRegistration *registration);

/**
 * @brief 通知客户端上一次 start_tx 已发送完成（通常在 UART TX 完成中断里调用）。
 * @param client 传 ATClient 地址（如 &at）；传 NULL 时什么也不做。
 * @return 无
 * @par 示例
 * @code
 * void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
 * {
 *     if (huart->Instance == USART2) {
 *         ATClient_OnTxComplete(&at);
 *     }
 * }
 * @endcode
 */
void ATClient_OnTxComplete(ATClient *client);

/**
 * @brief 通知客户端底层传输出错（中断安全，只置标志）。
 * @details 下一次 ATClient_Process() 会把当前请求以 TRANSPORT_ERROR 结束。
 * @param client 传 ATClient 地址（如 &at）；传 NULL 时什么也不做。
 * @return 无
 * @par 示例
 * @code
 * void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
 * {
 *     ATClient_OnTransportError(&at);
 * }
 * @endcode
 */
void ATClient_OnTransportError(ATClient *client);

/**
 * @brief 主循环处理函数：启动队首请求、解析接收数据、处理提示符与超时。
 * @details 非阻塞，不能在中断中调用；所有回调都在本函数内触发。
 * @param client 传 ATClient 地址（如 &at）；传 NULL 时什么也不做。
 * @return 无
 * @par 示例
 * @code
 * while (1) {
 *     ATClient_Process(&at);
 *     ML307C_Process(&modem);
 * }
 * @endcode
 */
void ATClient_Process(ATClient *client);

/**
 * @brief 判断客户端是否空闲（无活动请求且队列为空）。
 * @param client 传已用 ATClient_Init() 初始化的 ATClient 地址（如 &at）。
 * @return true 空闲；false 忙或 client 为 NULL。
 * @par 示例
 * @code
 * if (ATClient_IsIdle(&at)) {
 *     enter_low_power();
 * }
 * @endcode
 */
bool ATClient_IsIdle(const ATClient *client);

/**
 * @brief 获取运行期错误统计。
 * @param client 传已用 ATClient_Init() 初始化的 ATClient 地址（如 &at）。
 * @return 指向内部统计结构的只读指针；client 为 NULL 时返回 NULL。
 * @par 示例
 * @code
 * const ATClient_Stats *stats = ATClient_GetStats(&at);
 * if (stats != NULL && stats->rx_overflow_count > 0U) {
 *     printf("rx dropped %lu bytes\r\n", (unsigned long)stats->rx_dropped_bytes);
 * }
 * @endcode
 */
const ATClient_Stats *ATClient_GetStats(const ATClient *client);

/**
 * @brief 通过传输接口读取当前毫秒时间。
 * @param client 传已用 ATClient_Init() 初始化的 ATClient 地址（如 &at）。
 * @return 当前毫秒数；client 为 NULL 时返回 0。
 * @par 示例
 * @code
 * uint32_t started = ATClient_NowMs(&at);
 * @endcode
 */
uint32_t ATClient_NowMs(const ATClient *client);

/**
 * @brief 查询接收环形缓冲区中尚未解析的字节数。
 * @param client 传已用 ATClient_Init() 初始化的 ATClient 地址（如 &at）。
 * @return 待解析字节数；client 为 NULL 时返回 0。
 * @par 示例
 * @code
 * if (ATClient_RxPending(&at) > 0U) {
 *     ATClient_Process(&at);
 * }
 * @endcode
 */
size_t ATClient_RxPending(const ATClient *client);

#ifdef __cplusplus
}
#endif

#endif
