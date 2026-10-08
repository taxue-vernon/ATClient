/**
 * @file  ATClient.c
 * @brief 通用非阻塞 AT 指令客户端实现。公开接口的形参说明与示例见 ATClient.h。
 */

#include "ATClient.h"

#include <string.h>

static void ATClient_StartNext(ATClient *client);
static void ATClient_StartPayload(ATClient *client);
static void ATClient_ProcessRx(ATClient *client);
static void ATClient_HandleLine(ATClient *client, const char *line, size_t length);
static void ATClient_Complete(ATClient *client, ATClient_Result result);
static bool ATClient_LineEquals(const char *line, size_t length, const char *token);
static bool ATClient_DispatchUrc(ATClient *client, const char *line, size_t length);
static void ATClient_CheckLengthFrame(ATClient *client);
static void ATClient_ProcessFramePayload(ATClient *client);

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
bool ATClient_Init(ATClient *client, const ATClient_Transport *transport)
{
    if (client == NULL || transport == NULL || transport->start_tx == NULL ||
        transport->get_time_ms == NULL) {
        return false;
    }

    memset(client, 0, sizeof(*client));
    client->transport = *transport;
    client->state = ATCLIENT_STATE_IDLE;
    return true;
}

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
ATClient_Result ATClient_Submit(ATClient *client, const ATClient_Request *request)
{
    size_t insert_at;

    if (client == NULL || request == NULL || request->build_command == NULL ||
        request->timeout_ms == 0U ||
        (request->priority != ATCLIENT_PRIORITY_NORMAL &&
         request->priority != ATCLIENT_PRIORITY_URGENT) ||
        (request->prompt_byte == 0U &&
         (request->payload != NULL || request->payload_length != 0U)) ||
        (request->prompt_byte != 0U &&
         (request->payload == NULL || request->payload_length == 0U)) ||
        request->success_token_count > ATCLIENT_SUCCESS_TOKEN_CAPACITY ||
        request->failure_token_count > ATCLIENT_FAILURE_TOKEN_CAPACITY) {
        return ATCLIENT_RESULT_INVALID_ARGUMENT;
    }

    for (insert_at = 0U; insert_at < request->success_token_count; ++insert_at) {
        if (request->success_tokens[insert_at] == NULL ||
            request->success_tokens[insert_at][0] == '\0') {
            return ATCLIENT_RESULT_INVALID_ARGUMENT;
        }
    }
    for (insert_at = 0U; insert_at < request->failure_token_count; ++insert_at) {
        if (request->failure_tokens[insert_at] == NULL ||
            request->failure_tokens[insert_at][0] == '\0') {
            return ATCLIENT_RESULT_INVALID_ARGUMENT;
        }
    }

    if (client->queue_count >= ATCLIENT_QUEUE_CAPACITY) {
        ++client->stats.queue_full_count;
        return ATCLIENT_RESULT_QUEUE_FULL;
    }

    insert_at = client->queue_count;
    if (request->priority == ATCLIENT_PRIORITY_URGENT) {
        insert_at = 0U;
        while (insert_at < client->queue_count &&
               client->queue[insert_at].priority == ATCLIENT_PRIORITY_URGENT) {
            ++insert_at;
        }
    }

    if (insert_at < client->queue_count) {
        memmove(&client->queue[insert_at + 1U], &client->queue[insert_at],
                (client->queue_count - insert_at) * sizeof(client->queue[0]));
    }
    client->queue[insert_at] = *request;
    ++client->queue_count;
    return ATCLIENT_RESULT_OK;
}

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
                                     ATClient_UrcCallback callback, void *context)
{
    size_t prefix_length;
    size_t index;

    if (client == NULL || prefix == NULL || callback == NULL) {
        return ATCLIENT_RESULT_INVALID_ARGUMENT;
    }

    prefix_length = strlen(prefix);
    if (prefix_length == 0U || prefix_length >= ATCLIENT_LINE_CAPACITY) {
        return ATCLIENT_RESULT_INVALID_ARGUMENT;
    }

    for (index = 0U; index < client->urc_count; ++index) {
        if (client->urcs[index].prefix_length == prefix_length &&
            memcmp(client->urcs[index].prefix, prefix, prefix_length) == 0) {
            client->urcs[index].prefix = prefix;
            client->urcs[index].callback = callback;
            client->urcs[index].context = context;
            return ATCLIENT_RESULT_OK;
        }
    }

    if (client->urc_count >= ATCLIENT_URC_CAPACITY) {
        return ATCLIENT_RESULT_QUEUE_FULL;
    }

    client->urcs[client->urc_count].prefix = prefix;
    client->urcs[client->urc_count].prefix_length = prefix_length;
    client->urcs[client->urc_count].callback = callback;
    client->urcs[client->urc_count].context = context;
    ++client->urc_count;
    return ATCLIENT_RESULT_OK;
}

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
                                             const ATClient_LengthFrameRegistration *registration)
{
    size_t prefix_length;
    size_t index;

    if (client == NULL || registration == NULL || registration->prefix == NULL ||
        registration->probe == NULL || registration->on_begin == NULL ||
        registration->on_data == NULL || registration->on_end == NULL) {
        return ATCLIENT_RESULT_INVALID_ARGUMENT;
    }

    prefix_length = strlen(registration->prefix);
    if (prefix_length == 0U || prefix_length >= ATCLIENT_LINE_CAPACITY) {
        return ATCLIENT_RESULT_INVALID_ARGUMENT;
    }

    for (index = 0U; index < client->length_frame_count; ++index) {
        ATClient_LengthFrameSlot *slot = &client->length_frames[index];

        if (slot->prefix_length == prefix_length &&
            memcmp(slot->registration.prefix, registration->prefix, prefix_length) == 0) {
            slot->registration = *registration;
            slot->prefix_length = prefix_length;
            return ATCLIENT_RESULT_OK;
        }
    }

    if (client->length_frame_count >= ATCLIENT_LENGTH_FRAME_CAPACITY) {
        return ATCLIENT_RESULT_QUEUE_FULL;
    }

    client->length_frames[client->length_frame_count].registration = *registration;
    client->length_frames[client->length_frame_count].prefix_length = prefix_length;
    ++client->length_frame_count;
    return ATCLIENT_RESULT_OK;
}

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
ATClient_Result ATClient_PushRx(ATClient *client, const uint8_t *data, size_t length)
{
    size_t copied = 0U;
    uint32_t head;
    uint32_t tail;

    if (client == NULL || (data == NULL && length != 0U)) {
        return ATCLIENT_RESULT_INVALID_ARGUMENT;
    }

    head = client->rx_head;
    tail = client->rx_tail;
    while (copied < length && (uint32_t)(head - tail) < ATCLIENT_RX_CAPACITY) {
        client->rx_buffer[head % ATCLIENT_RX_CAPACITY] = data[copied];
        ++head;
        ++copied;
    }
    client->rx_head = head;

    if (copied != length) {
        ++client->stats.rx_overflow_count;
        client->stats.rx_dropped_bytes += (uint32_t)(length - copied);
        return ATCLIENT_RESULT_RX_OVERFLOW;
    }

    return ATCLIENT_RESULT_OK;
}

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
void ATClient_OnTxComplete(ATClient *client)
{
    if (client != NULL && client->has_active_request && client->state == ATCLIENT_STATE_TX_ACTIVE) {
        bool wait_prompt = client->active_request.prompt_byte != 0U;
        client->state = wait_prompt ? ATCLIENT_STATE_WAIT_PROMPT : ATCLIENT_STATE_WAIT_RESPONSE;
    } else if (client != NULL && client->has_active_request &&
               client->state == ATCLIENT_STATE_PAYLOAD_TX_ACTIVE) {
        client->state = ATCLIENT_STATE_WAIT_RESPONSE;
    }
}

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
void ATClient_OnTransportError(ATClient *client)
{
    if (client != NULL) {
        client->transport_error_pending = true;
    }
}

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
void ATClient_Process(ATClient *client)
{
    uint32_t now_ms;

    if (client == NULL) {
        return;
    }

    if (!client->has_active_request) {
        ATClient_StartNext(client);
    }

    if (client->transport_error_pending) {
        client->transport_error_pending = false;
        ++client->stats.transport_error_count;
        if (client->has_active_request) {
            ATClient_Complete(client, ATCLIENT_RESULT_TRANSPORT_ERROR);
        }
    }

    ATClient_ProcessRx(client);

    if (client->has_active_request && client->state == ATCLIENT_STATE_WAIT_PROMPT &&
        client->prompt_seen) {
        ATClient_StartPayload(client);
    }

    if (client->has_active_request) {
        now_ms = client->transport.get_time_ms(client->transport.context);
        if ((uint32_t)(now_ms - client->started_ms) >= client->active_request.timeout_ms) {
            ATClient_Complete(client, ATCLIENT_RESULT_TIMEOUT);
        }
    }

    if (!client->has_active_request) {
        ATClient_StartNext(client);
    }
}

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
bool ATClient_IsIdle(const ATClient *client)
{
    return client != NULL && !client->has_active_request && client->queue_count == 0U;
}

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
const ATClient_Stats *ATClient_GetStats(const ATClient *client)
{
    return client == NULL ? NULL : &client->stats;
}

/**
 * @brief 通过传输接口读取当前毫秒时间。
 * @param client 传已用 ATClient_Init() 初始化的 ATClient 地址（如 &at）。
 * @return 当前毫秒数；client 为 NULL 时返回 0。
 * @par 示例
 * @code
 * uint32_t started = ATClient_NowMs(&at);
 * @endcode
 */
uint32_t ATClient_NowMs(const ATClient *client)
{
    if (client == NULL) {
        return 0U;
    }

    return client->transport.get_time_ms(client->transport.context);
}

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
size_t ATClient_RxPending(const ATClient *client)
{
    return client == NULL ? 0U : (size_t)(uint32_t)(client->rx_head - client->rx_tail);
}

/**
 * @brief 取出队首请求，构造命令并启动发送。
 * @details 已有活动请求或队列为空时直接返回；命令构造失败以 INVALID_ARGUMENT 结束，
 *          start_tx 失败以 TRANSPORT_ERROR 结束。命令末尾自动追加 "\r\n"。
 * @param client 传当前 ATClient 对象指针（内部调用，由公开接口传进来的 client，已保证非 NULL）。
 * @return 无
 * @par 示例
 * @code
 * // ATClient_Process() 内部：当前没有活动请求时启动下一条
 * if (!client->has_active_request) {
 *     ATClient_StartNext(client);
 * }
 * @endcode
 */
static void ATClient_StartNext(ATClient *client)
{
    size_t built_length = 0U;

    if (client->has_active_request || client->queue_count == 0U) {
        return;
    }

    client->active_request = client->queue[0];
    if (client->queue_count > 1U) {
        memmove(&client->queue[0], &client->queue[1],
                (client->queue_count - 1U) * sizeof(client->queue[0]));
    }
    --client->queue_count;
    client->has_active_request = true;
    client->state = ATCLIENT_STATE_IDLE;
    client->line_length = 0U;
    client->discarding_line = false;
    client->prompt_seen = false;
    client->skip_prompt_space = false;
    client->success_token_index = 0U;

    if (!client->active_request.build_command(client->active_request.command_context,
                                              (char *)client->command_buffer,
                                              ATCLIENT_COMMAND_CAPACITY, &built_length) ||
        built_length == 0U || built_length > ATCLIENT_COMMAND_CAPACITY) {
        ATClient_Complete(client, ATCLIENT_RESULT_INVALID_ARGUMENT);
        return;
    }

    client->command_buffer[built_length] = '\r';
    client->command_buffer[built_length + 1U] = '\n';
    client->command_length = built_length;
    client->started_ms = client->transport.get_time_ms(client->transport.context);

    /* Non-blocking: start the transport and return. Completion is reported
     * later through ATClient_OnTxComplete() and ATClient_Process(). */
    client->state = ATCLIENT_STATE_TX_ACTIVE;
    if (!client->transport.start_tx(client->transport.context, client->command_buffer,
                                    built_length + 2U)) {
        ++client->stats.transport_error_count;
        ATClient_Complete(client, ATCLIENT_RESULT_TRANSPORT_ERROR);
        return;
    }
}

/**
 * @brief 收到提示符（如 '>'）后启动 payload 发送。
 * @param client 传当前 ATClient 对象指针；调用前 active_request 中的 payload 必须有效。
 * @return 无
 * @par 示例
 * @code
 * if (client->state == ATCLIENT_STATE_WAIT_PROMPT && client->prompt_seen) {
 *     ATClient_StartPayload(client);
 * }
 * @endcode
 */
static void ATClient_StartPayload(ATClient *client)
{
    client->prompt_seen = false;

    /* Non-blocking: arm the payload transfer and return immediately. The
     * transport later reports completion through ATClient_OnTxComplete(). */
    client->state = ATCLIENT_STATE_PAYLOAD_TX_ACTIVE;
    if (!client->transport.start_tx(client->transport.context, client->active_request.payload,
                                    client->active_request.payload_length)) {
        ++client->stats.transport_error_count;
        ATClient_Complete(client, ATCLIENT_RESULT_TRANSPORT_ERROR);
        return;
    }
}

/**
 * @brief 消费接收环形缓冲区：识别提示符、拼接行、分发行或进入定长帧负载模式。
 * @details 超过 ATCLIENT_LINE_CAPACITY 的行会被丢弃到行尾，当前请求以 LINE_TOO_LONG 结束。
 * @param client 传当前 ATClient 对象指针（内部调用，由公开接口传进来的 client，已保证非 NULL）。
 * @return 无
 * @par 示例
 * @code
 * ATClient_ProcessRx(client);   // 在 ATClient_Process() 中每次调用
 * @endcode
 */
static void ATClient_ProcessRx(ATClient *client)
{
    while (client->rx_tail != client->rx_head) {
        if (client->parser_state == ATCLIENT_PARSER_FRAME_PAYLOAD) {
            ATClient_ProcessFramePayload(client);
            continue;
        }

        uint32_t tail = client->rx_tail;
        uint8_t byte = client->rx_buffer[tail % ATCLIENT_RX_CAPACITY];

        client->rx_tail = tail + 1U;

        if (client->skip_prompt_space) {
            client->skip_prompt_space = false;
            if (byte == (uint8_t)' ') {
                continue;
            }
        }

        if (client->has_active_request && client->active_request.prompt_byte != 0U &&
            (client->state == ATCLIENT_STATE_TX_ACTIVE ||
             client->state == ATCLIENT_STATE_WAIT_PROMPT) &&
            client->line_length == 0U && !client->discarding_line &&
            byte == client->active_request.prompt_byte) {
            /* Only a standalone prompt starts payload TX; '>' embedded in a
             * URC or response line is ordinary text. */
            client->prompt_seen = true;
            /* The optional space can be in the same RX batch as '>'.
             * Consume it now, before building the next response line. */
            client->skip_prompt_space = true;
            continue;
        }

        if (byte == '\r' || byte == '\n') {
            if (client->discarding_line) {
                client->discarding_line = false;
                client->line_length = 0U;
            } else if (client->line_length > 0U) {
                client->line_buffer[client->line_length] = '\0';
                ATClient_HandleLine(client, client->line_buffer, client->line_length);
                client->line_length = 0U;
            }
            continue;
        }

        if (client->discarding_line) {
            continue;
        }

        if (client->line_length >= ATCLIENT_LINE_CAPACITY - 1U) {
            ++client->stats.line_too_long_count;
            client->discarding_line = true;
            client->line_length = 0U;
            if (client->has_active_request) {
                ATClient_Complete(client, ATCLIENT_RESULT_LINE_TOO_LONG);
            }
            continue;
        }

        client->line_buffer[client->line_length++] = (char)byte;
        ATClient_CheckLengthFrame(client);
    }
}

/**
 * @brief 处理一条完整的接收行。
 * @details 优先级：命令回显 -> ERROR/+CME/+CMS -> 失败标记 -> 成功标记/OK -> URC -> on_line。
 * @param client 传当前 ATClient 对象指针（内部调用，由公开接口传进来的 client，已保证非 NULL）。
 * @param line   传刚拼好的一行（client->line_buffer），以 '\0' 结尾、不含 "\r
 *               "。
 * @param length 传这一行的字节数（client->line_length）。
 * @return 无
 * @par 示例
 * @code
 * client->line_buffer[client->line_length] = '\0';
 * ATClient_HandleLine(client, client->line_buffer, client->line_length);
 * @endcode
 */
static void ATClient_HandleLine(ATClient *client, const char *line, size_t length)
{
    size_t index;

    if (client->has_active_request && length == client->command_length &&
        memcmp(line, client->command_buffer, length) == 0) {
        return;
    }

    if (client->has_active_request) {
        if (length == 5U && memcmp(line, "ERROR", 5U) == 0) {
            ATClient_Complete(client, ATCLIENT_RESULT_ERROR_RESPONSE);
            return;
        }
        if (length >= 11U && memcmp(line, "+CME ERROR:", 11U) == 0) {
            ATClient_Complete(client, ATCLIENT_RESULT_CME_ERROR);
            return;
        }
        if (length >= 11U && memcmp(line, "+CMS ERROR:", 11U) == 0) {
            ATClient_Complete(client, ATCLIENT_RESULT_CMS_ERROR);
            return;
        }

        for (index = 0U; index < client->active_request.failure_token_count; ++index) {
            if (ATClient_LineEquals(line, length, client->active_request.failure_tokens[index])) {
                ATClient_Complete(client, ATCLIENT_RESULT_ERROR_RESPONSE);
                return;
            }
        }

        if (client->active_request.success_token_count > 0U &&
            ATClient_LineEquals(
                line, length, client->active_request.success_tokens[client->success_token_index])) {
            ++client->success_token_index;
            if (client->success_token_index >= client->active_request.success_token_count) {
                ATClient_Complete(client, ATCLIENT_RESULT_OK);
            }
            return;
        }

        if (client->active_request.success_token_count == 0U && length == 2U &&
            memcmp(line, "OK", 2U) == 0) {
            ATClient_Complete(client, ATCLIENT_RESULT_OK);
            return;
        }
    }

    if (ATClient_DispatchUrc(client, line, length)) {
        return;
    }

    if (client->has_active_request && client->active_request.on_line != NULL) {
        client->active_request.on_line(client->active_request.callback_context, line, length);
    }
}

/**
 * @brief 结束当前活动请求，复位状态并回调 on_complete。
 * @details 失败时若正处于定长帧负载接收，会先以同样的结果调用 on_end 释放该帧。
 * @param client 传当前 ATClient 对象指针（内部调用，由公开接口传进来的 client，已保证非 NULL）。
 * @param result 传要报告给 on_complete 的结果，如 ATCLIENT_RESULT_OK、ATCLIENT_RESULT_TIMEOUT。
 * @return 无
 * @par 示例
 * @code
 * ATClient_Complete(client, ATCLIENT_RESULT_TIMEOUT);
 * @endcode
 */
static void ATClient_Complete(ATClient *client, ATClient_Result result)
{
    ATClient_CompleteCallback callback;
    void *context;

    if (!client->has_active_request) {
        return;
    }

    callback = client->active_request.on_complete;
    context = client->active_request.callback_context;
    /* A failed transaction must release a truncated length frame. Otherwise
     * the next command's OK would be consumed as the old binary payload. */
    if (result != ATCLIENT_RESULT_OK) {
        if (client->parser_state == ATCLIENT_PARSER_FRAME_PAYLOAD) {
            ATClient_LengthFrameSlot *slot = &client->length_frames[client->active_length_frame];
            client->parser_state = ATCLIENT_PARSER_LINE;
            client->frame_payload_remaining = 0U;
            slot->registration.on_end(slot->registration.context, result);
        }
        client->line_length = 0U;
        client->discarding_line = false;
    }
    memset(&client->active_request, 0, sizeof(client->active_request));
    client->has_active_request = false;
    client->state = ATCLIENT_STATE_IDLE;
    client->command_length = 0U;
    client->prompt_seen = false;
    client->skip_prompt_space = false;
    client->success_token_index = 0U;

    if (callback != NULL) {
        callback(context, result);
    }
}

/**
 * @brief 判断行内容是否与标记字符串完全相等。
 * @param line   传要比较的行内容。
 * @param length 传 line 的字节数。
 * @param token  传以 '\0' 结尾的标记字符串，如 "OK"、"SEND OK"。
 * @return true 完全相等；false 不相等。
 * @par 示例
 * @code
 * if (ATClient_LineEquals(line, length, "SEND OK")) {
 *     // 匹配成功
 * }
 * @endcode
 */
static bool ATClient_LineEquals(const char *line, size_t length, const char *token)
{
    size_t token_length = strlen(token);

    return length == token_length && memcmp(line, token, length) == 0;
}

/**
 * @brief 按注册的前缀查找并调用 URC 回调。
 * @param client 传当前 ATClient 对象指针（内部调用，由公开接口传进来的 client，已保证非 NULL）。
 * @param line   传要匹配的行内容。
 * @param length 传 line 的字节数。
 * @return true 已作为 URC 处理；false 没有匹配的前缀。
 * @par 示例
 * @code
 * if (ATClient_DispatchUrc(client, line, length)) {
 *     return;   // URC 不再交给 on_line
 * }
 * @endcode
 */
static bool ATClient_DispatchUrc(ATClient *client, const char *line, size_t length)
{
    size_t index;

    for (index = 0U; index < client->urc_count; ++index) {
        const ATClient_UrcRegistration *registration = &client->urcs[index];

        if (length >= registration->prefix_length &&
            memcmp(line, registration->prefix, registration->prefix_length) == 0) {
            registration->callback(registration->context, line, length);
            return true;
        }
    }

    return false;
}

/**
 * @brief 用当前未完成的行依次调用已注册的帧头探测回调。
 * @details 探测到 READY 时切换为负载接收模式并调用 on_begin；ERROR 时丢弃该行。
 * @param client 传当前 ATClient 对象指针（内部调用，由公开接口传进来的 client，已保证非 NULL）。
 * @return 无
 * @par 示例
 * @code
 * client->line_buffer[client->line_length++] = (char)byte;
 * ATClient_CheckLengthFrame(client);
 * @endcode
 */
static void ATClient_CheckLengthFrame(ATClient *client)
{
    size_t index;

    for (index = 0U; index < client->length_frame_count; ++index) {
        ATClient_LengthFrameSlot *slot = &client->length_frames[index];
        ATClient_FrameProbeResult result;
        size_t payload_length = 0U;

        result = slot->registration.probe(slot->registration.context,
                                          (const uint8_t *)client->line_buffer, client->line_length,
                                          &payload_length);

        if (result == ATCLIENT_FRAME_READY) {
            if (payload_length > ATCLIENT_FRAME_PAYLOAD_MAX) {
                ++client->stats.protocol_error_count;
                client->discarding_line = true;
                client->line_length = 0U;
                return;
            }

            client->active_length_frame = index;
            client->frame_payload_remaining = payload_length;
            client->parser_state = ATCLIENT_PARSER_FRAME_PAYLOAD;
            slot->registration.on_begin(slot->registration.context,
                                        (const uint8_t *)client->line_buffer, client->line_length,
                                        payload_length);
            client->line_length = 0U;

            if (payload_length == 0U) {
                slot->registration.on_end(slot->registration.context, ATCLIENT_RESULT_OK);
                client->parser_state = ATCLIENT_PARSER_LINE;
            }
            return;
        }

        if (result == ATCLIENT_FRAME_ERROR) {
            ++client->stats.protocol_error_count;
            client->discarding_line = true;
            client->line_length = 0U;
            return;
        }
    }
}

/**
 * @brief 把环形缓冲区中连续可读的负载字节交给 on_data，收齐后调用 on_end。
 * @param client 传当前 ATClient 对象指针；调用前 parser_state 必须是
 *               ATCLIENT_PARSER_FRAME_PAYLOAD。
 * @return 无
 * @par 示例
 * @code
 * if (client->parser_state == ATCLIENT_PARSER_FRAME_PAYLOAD) {
 *     ATClient_ProcessFramePayload(client);
 * }
 * @endcode
 */
static void ATClient_ProcessFramePayload(ATClient *client)
{
    ATClient_LengthFrameSlot *slot = &client->length_frames[client->active_length_frame];
    uint32_t tail = client->rx_tail;
    size_t available = (size_t)(uint32_t)(client->rx_head - client->rx_tail);
    size_t tail_index = (size_t)(tail % ATCLIENT_RX_CAPACITY);
    size_t contiguous = ATCLIENT_RX_CAPACITY - tail_index;

    if (contiguous > available) {
        contiguous = available;
    }
    if (contiguous > client->frame_payload_remaining) {
        contiguous = client->frame_payload_remaining;
    }

    if (contiguous > 0U) {
        slot->registration.on_data(slot->registration.context, &client->rx_buffer[tail_index],
                                   contiguous);
        client->rx_tail = tail + (uint32_t)contiguous;
        client->frame_payload_remaining -= contiguous;
    }

    if (client->frame_payload_remaining == 0U) {
        slot->registration.on_end(slot->registration.context, ATCLIENT_RESULT_OK);
        client->parser_state = ATCLIENT_PARSER_LINE;
    }
}
