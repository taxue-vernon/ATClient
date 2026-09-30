#include "ATClient.h"

#include <string.h>

static void ATClient_StartNext(ATClient *client);
static void ATClient_StartPayload(ATClient *client);
static void ATClient_ProcessRx(ATClient *client);
static void ATClient_HandleLine(ATClient *client,
                                const char *line,
                                size_t length);
static void ATClient_Complete(ATClient *client, ATClient_Result result);
static bool ATClient_LineEquals(const char *line,
                                size_t length,
                                const char *token);
static bool ATClient_DispatchUrc(ATClient *client,
                                 const char *line,
                                 size_t length);
static void ATClient_CheckLengthFrame(ATClient *client);
static void ATClient_ProcessFramePayload(ATClient *client);

bool ATClient_Init(ATClient *client, const ATClient_Transport *transport)
{
    if (client == NULL || transport == NULL || transport->start_tx == NULL ||
        transport->get_time_ms == NULL)
    {
        return false;
    }

    memset(client, 0, sizeof(*client));
    client->transport = *transport;
    client->state = ATCLIENT_STATE_IDLE;
    return true;
}

ATClient_Result ATClient_Submit(ATClient *client,
                                const ATClient_Request *request)
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
        request->failure_token_count > ATCLIENT_FAILURE_TOKEN_CAPACITY)
    {
        return ATCLIENT_RESULT_INVALID_ARGUMENT;
    }

    for (insert_at = 0U; insert_at < request->success_token_count; ++insert_at)
    {
        if (request->success_tokens[insert_at] == NULL ||
            request->success_tokens[insert_at][0] == '\0')
        {
            return ATCLIENT_RESULT_INVALID_ARGUMENT;
        }
    }
    for (insert_at = 0U; insert_at < request->failure_token_count; ++insert_at)
    {
        if (request->failure_tokens[insert_at] == NULL ||
            request->failure_tokens[insert_at][0] == '\0')
        {
            return ATCLIENT_RESULT_INVALID_ARGUMENT;
        }
    }

    if (client->queue_count >= ATCLIENT_QUEUE_CAPACITY)
    {
        ++client->stats.queue_full_count;
        return ATCLIENT_RESULT_QUEUE_FULL;
    }

    insert_at = client->queue_count;
    if (request->priority == ATCLIENT_PRIORITY_URGENT)
    {
        insert_at = 0U;
        while (insert_at < client->queue_count &&
               client->queue[insert_at].priority == ATCLIENT_PRIORITY_URGENT)
        {
            ++insert_at;
        }
    }

    if (insert_at < client->queue_count)
    {
        memmove(&client->queue[insert_at + 1U],
                &client->queue[insert_at],
                (client->queue_count - insert_at) * sizeof(client->queue[0]));
    }
    client->queue[insert_at] = *request;
    ++client->queue_count;
    return ATCLIENT_RESULT_OK;
}

ATClient_Result ATClient_RegisterUrc(ATClient *client,
                                     const char *prefix,
                                     ATClient_UrcCallback callback,
                                     void *context)
{
    size_t prefix_length;
    size_t index;

    if (client == NULL || prefix == NULL || callback == NULL)
    {
        return ATCLIENT_RESULT_INVALID_ARGUMENT;
    }

    prefix_length = strlen(prefix);
    if (prefix_length == 0U || prefix_length >= ATCLIENT_LINE_CAPACITY)
    {
        return ATCLIENT_RESULT_INVALID_ARGUMENT;
    }

    for (index = 0U; index < client->urc_count; ++index)
    {
        if (client->urcs[index].prefix_length == prefix_length &&
            memcmp(client->urcs[index].prefix, prefix, prefix_length) == 0)
        {
            client->urcs[index].prefix = prefix;
            client->urcs[index].callback = callback;
            client->urcs[index].context = context;
            return ATCLIENT_RESULT_OK;
        }
    }

    if (client->urc_count >= ATCLIENT_URC_CAPACITY)
    {
        return ATCLIENT_RESULT_QUEUE_FULL;
    }

    client->urcs[client->urc_count].prefix = prefix;
    client->urcs[client->urc_count].prefix_length = prefix_length;
    client->urcs[client->urc_count].callback = callback;
    client->urcs[client->urc_count].context = context;
    ++client->urc_count;
    return ATCLIENT_RESULT_OK;
}

ATClient_Result ATClient_RegisterLengthFrame(
    ATClient *client,
    const ATClient_LengthFrameRegistration *registration)
{
    size_t prefix_length;
    size_t index;

    if (client == NULL || registration == NULL ||
        registration->prefix == NULL || registration->probe == NULL ||
        registration->on_begin == NULL || registration->on_data == NULL ||
        registration->on_end == NULL)
    {
        return ATCLIENT_RESULT_INVALID_ARGUMENT;
    }

    prefix_length = strlen(registration->prefix);
    if (prefix_length == 0U || prefix_length >= ATCLIENT_LINE_CAPACITY)
    {
        return ATCLIENT_RESULT_INVALID_ARGUMENT;
    }

    for (index = 0U; index < client->length_frame_count; ++index)
    {
        ATClient_LengthFrameSlot *slot = &client->length_frames[index];

        if (slot->prefix_length == prefix_length &&
            memcmp(slot->registration.prefix,
                   registration->prefix,
                   prefix_length) == 0)
        {
            slot->registration = *registration;
            slot->prefix_length = prefix_length;
            return ATCLIENT_RESULT_OK;
        }
    }

    if (client->length_frame_count >= ATCLIENT_LENGTH_FRAME_CAPACITY)
    {
        return ATCLIENT_RESULT_QUEUE_FULL;
    }

    client->length_frames[client->length_frame_count].registration =
        *registration;
    client->length_frames[client->length_frame_count].prefix_length =
        prefix_length;
    ++client->length_frame_count;
    return ATCLIENT_RESULT_OK;
}

ATClient_Result ATClient_PushRx(ATClient *client,
                                const uint8_t *data,
                                size_t length)
{
    size_t copied = 0U;
    uint32_t head;
    uint32_t tail;

    if (client == NULL || (data == NULL && length != 0U))
    {
        return ATCLIENT_RESULT_INVALID_ARGUMENT;
    }

    head = client->rx_head;
    tail = client->rx_tail;
    while (copied < length &&
           (uint32_t)(head - tail) < ATCLIENT_RX_CAPACITY)
    {
        client->rx_buffer[head % ATCLIENT_RX_CAPACITY] = data[copied];
        ++head;
        ++copied;
    }
    client->rx_head = head;

    if (copied != length)
    {
        ++client->stats.rx_overflow_count;
        client->stats.rx_dropped_bytes += (uint32_t)(length - copied);
        return ATCLIENT_RESULT_RX_OVERFLOW;
    }

    return ATCLIENT_RESULT_OK;
}

void ATClient_OnTxComplete(ATClient *client)
{
    if (client != NULL && client->has_active_request &&
        client->state == ATCLIENT_STATE_TX_ACTIVE)
    {
        client->state = client->active_request.prompt_byte == 0U
                            ? ATCLIENT_STATE_WAIT_RESPONSE
                            : ATCLIENT_STATE_WAIT_PROMPT;
    }
    else if (client != NULL && client->has_active_request &&
             client->state == ATCLIENT_STATE_PAYLOAD_TX_ACTIVE)
    {
        client->state = ATCLIENT_STATE_WAIT_RESPONSE;
    }
}

void ATClient_OnTransportError(ATClient *client)
{
    if (client != NULL)
    {
        client->transport_error_pending = true;
    }
}

void ATClient_Process(ATClient *client)
{
    uint32_t now_ms;

    if (client == NULL)
    {
        return;
    }

    if (!client->has_active_request)
    {
        ATClient_StartNext(client);
    }

    if (client->transport_error_pending)
    {
        client->transport_error_pending = false;
        ++client->stats.transport_error_count;
        if (client->has_active_request)
        {
            ATClient_Complete(client, ATCLIENT_RESULT_TRANSPORT_ERROR);
        }
    }

    ATClient_ProcessRx(client);

    if (client->has_active_request &&
        client->state == ATCLIENT_STATE_WAIT_PROMPT && client->prompt_seen)
    {
        ATClient_StartPayload(client);
    }

    if (client->has_active_request)
    {
        now_ms = client->transport.get_time_ms(client->transport.context);
        if ((uint32_t)(now_ms - client->started_ms) >=
            client->active_request.timeout_ms)
        {
            ATClient_Complete(client, ATCLIENT_RESULT_TIMEOUT);
        }
    }

    if (!client->has_active_request)
    {
        ATClient_StartNext(client);
    }
}

bool ATClient_IsIdle(const ATClient *client)
{
    return client != NULL && !client->has_active_request &&
           client->queue_count == 0U;
}

const ATClient_Stats *ATClient_GetStats(const ATClient *client)
{
    return client == NULL ? NULL : &client->stats;
}

uint32_t ATClient_NowMs(const ATClient *client)
{
    if (client == NULL)
    {
        return 0U;
    }

    return client->transport.get_time_ms(client->transport.context);
}

size_t ATClient_RxPending(const ATClient *client)
{
    return client == NULL
               ? 0U
               : (size_t)(uint32_t)(client->rx_head - client->rx_tail);
}

static void ATClient_StartNext(ATClient *client)
{
    size_t built_length = 0U;

    if (client->has_active_request || client->queue_count == 0U)
    {
        return;
    }

    client->active_request = client->queue[0];
    if (client->queue_count > 1U)
    {
        memmove(&client->queue[0],
                &client->queue[1],
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

    if (!client->active_request.build_command(
            client->active_request.command_context,
            (char *)client->command_buffer,
            ATCLIENT_COMMAND_CAPACITY,
            &built_length) ||
        built_length == 0U || built_length > ATCLIENT_COMMAND_CAPACITY)
    {
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
    if (!client->transport.start_tx(client->transport.context,
                                    client->command_buffer,
                                    built_length + 2U))
    {
        ++client->stats.transport_error_count;
        ATClient_Complete(client, ATCLIENT_RESULT_TRANSPORT_ERROR);
        return;
    }

}

static void ATClient_StartPayload(ATClient *client)
{
    client->prompt_seen = false;

    /* Non-blocking: arm the payload transfer and return immediately. The
     * transport later reports completion through ATClient_OnTxComplete(). */
    client->state = ATCLIENT_STATE_PAYLOAD_TX_ACTIVE;
    if (!client->transport.start_tx(client->transport.context,
                                    client->active_request.payload,
                                    client->active_request.payload_length))
    {
        ++client->stats.transport_error_count;
        ATClient_Complete(client, ATCLIENT_RESULT_TRANSPORT_ERROR);
        return;
    }

}

static void ATClient_ProcessRx(ATClient *client)
{
    while (client->rx_tail != client->rx_head)
    {
        if (client->parser_state == ATCLIENT_PARSER_FRAME_PAYLOAD)
        {
            ATClient_ProcessFramePayload(client);
            continue;
        }

        uint32_t tail = client->rx_tail;
        uint8_t byte = client->rx_buffer[tail % ATCLIENT_RX_CAPACITY];

        client->rx_tail = tail + 1U;

        if (client->skip_prompt_space)
        {
            client->skip_prompt_space = false;
            if (byte == (uint8_t)' ')
            {
                continue;
            }
        }

        if (client->has_active_request &&
            client->active_request.prompt_byte != 0U &&
            (client->state == ATCLIENT_STATE_TX_ACTIVE ||
             client->state == ATCLIENT_STATE_WAIT_PROMPT) &&
            client->line_length == 0U && !client->discarding_line &&
            byte == client->active_request.prompt_byte)
        {
            /* Only a standalone prompt starts payload TX; '>' embedded in a
             * URC or response line is ordinary text. */
            client->prompt_seen = true;
            /* The optional space can be in the same RX batch as '>'.
             * Consume it now, before building the next response line. */
            client->skip_prompt_space = true;
            continue;
        }

        if (byte == '\r' || byte == '\n')
        {
            if (client->discarding_line)
            {
                client->discarding_line = false;
                client->line_length = 0U;
            }
            else if (client->line_length > 0U)
            {
                client->line_buffer[client->line_length] = '\0';
                ATClient_HandleLine(client,
                                    client->line_buffer,
                                    client->line_length);
                client->line_length = 0U;
            }
            continue;
        }

        if (client->discarding_line)
        {
            continue;
        }

        if (client->line_length >= ATCLIENT_LINE_CAPACITY - 1U)
        {
            ++client->stats.line_too_long_count;
            client->discarding_line = true;
            client->line_length = 0U;
            if (client->has_active_request)
            {
                ATClient_Complete(client, ATCLIENT_RESULT_LINE_TOO_LONG);
            }
            continue;
        }

        client->line_buffer[client->line_length++] = (char)byte;
        ATClient_CheckLengthFrame(client);
    }
}

static void ATClient_HandleLine(ATClient *client,
                                const char *line,
                                size_t length)
{
    size_t index;

    if (client->has_active_request && length == client->command_length &&
        memcmp(line, client->command_buffer, length) == 0)
    {
        return;
    }

    if (client->has_active_request)
    {
        if (length == 5U && memcmp(line, "ERROR", 5U) == 0)
        {
            ATClient_Complete(client, ATCLIENT_RESULT_ERROR_RESPONSE);
            return;
        }
        if (length >= 11U && memcmp(line, "+CME ERROR:", 11U) == 0)
        {
            ATClient_Complete(client, ATCLIENT_RESULT_CME_ERROR);
            return;
        }
        if (length >= 11U && memcmp(line, "+CMS ERROR:", 11U) == 0)
        {
            ATClient_Complete(client, ATCLIENT_RESULT_CMS_ERROR);
            return;
        }

        for (index = 0U;
             index < client->active_request.failure_token_count;
             ++index)
        {
            if (ATClient_LineEquals(line,
                                    length,
                                    client->active_request.failure_tokens[index]))
            {
                ATClient_Complete(client, ATCLIENT_RESULT_ERROR_RESPONSE);
                return;
            }
        }

        if (client->active_request.success_token_count > 0U &&
            ATClient_LineEquals(
                line,
                length,
                client->active_request
                    .success_tokens[client->success_token_index]))
        {
            ++client->success_token_index;
            if (client->success_token_index >=
                client->active_request.success_token_count)
            {
                ATClient_Complete(client, ATCLIENT_RESULT_OK);
            }
            return;
        }

        if (client->active_request.success_token_count == 0U &&
            length == 2U && memcmp(line, "OK", 2U) == 0)
        {
            ATClient_Complete(client, ATCLIENT_RESULT_OK);
            return;
        }
    }

    if (ATClient_DispatchUrc(client, line, length))
    {
        return;
    }

    if (client->has_active_request && client->active_request.on_line != NULL)
    {
        client->active_request.on_line(client->active_request.callback_context,
                                       line,
                                       length);
    }
}

static void ATClient_Complete(ATClient *client, ATClient_Result result)
{
    ATClient_CompleteCallback callback;
    void *context;

    if (!client->has_active_request)
    {
        return;
    }

    callback = client->active_request.on_complete;
    context = client->active_request.callback_context;
    /* A failed transaction must release a truncated length frame. Otherwise
     * the next command's OK would be consumed as the old binary payload. */
    if (result != ATCLIENT_RESULT_OK)
    {
        if (client->parser_state == ATCLIENT_PARSER_FRAME_PAYLOAD)
        {
            ATClient_LengthFrameSlot *slot =
                &client->length_frames[client->active_length_frame];
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

    if (callback != NULL)
    {
        callback(context, result);
    }
}

static bool ATClient_LineEquals(const char *line,
                                size_t length,
                                const char *token)
{
    size_t token_length = strlen(token);

    return length == token_length && memcmp(line, token, length) == 0;
}

static bool ATClient_DispatchUrc(ATClient *client,
                                 const char *line,
                                 size_t length)
{
    size_t index;

    for (index = 0U; index < client->urc_count; ++index)
    {
        const ATClient_UrcRegistration *registration = &client->urcs[index];

        if (length >= registration->prefix_length &&
            memcmp(line, registration->prefix, registration->prefix_length) == 0)
        {
            registration->callback(registration->context, line, length);
            return true;
        }
    }

    return false;
}

static void ATClient_CheckLengthFrame(ATClient *client)
{
    size_t index;

    for (index = 0U; index < client->length_frame_count; ++index)
    {
        ATClient_LengthFrameSlot *slot = &client->length_frames[index];
        ATClient_FrameProbeResult result;
        size_t payload_length = 0U;

        result = slot->registration.probe(
            slot->registration.context,
            (const uint8_t *)client->line_buffer,
            client->line_length,
            &payload_length);

        if (result == ATCLIENT_FRAME_READY)
        {
            if (payload_length > ATCLIENT_FRAME_PAYLOAD_MAX)
            {
                ++client->stats.protocol_error_count;
                client->discarding_line = true;
                client->line_length = 0U;
                return;
            }

            client->active_length_frame = index;
            client->frame_payload_remaining = payload_length;
            client->parser_state = ATCLIENT_PARSER_FRAME_PAYLOAD;
            slot->registration.on_begin(slot->registration.context,
                                        (const uint8_t *)client->line_buffer,
                                        client->line_length,
                                        payload_length);
            client->line_length = 0U;

            if (payload_length == 0U)
            {
                slot->registration.on_end(slot->registration.context,
                                          ATCLIENT_RESULT_OK);
                client->parser_state = ATCLIENT_PARSER_LINE;
            }
            return;
        }

        if (result == ATCLIENT_FRAME_ERROR)
        {
            ++client->stats.protocol_error_count;
            client->discarding_line = true;
            client->line_length = 0U;
            return;
        }
    }
}

static void ATClient_ProcessFramePayload(ATClient *client)
{
    ATClient_LengthFrameSlot *slot =
        &client->length_frames[client->active_length_frame];
    uint32_t tail = client->rx_tail;
    size_t available =
        (size_t)(uint32_t)(client->rx_head - client->rx_tail);
    size_t tail_index = (size_t)(tail % ATCLIENT_RX_CAPACITY);
    size_t contiguous = ATCLIENT_RX_CAPACITY - tail_index;

    if (contiguous > available)
    {
        contiguous = available;
    }
    if (contiguous > client->frame_payload_remaining)
    {
        contiguous = client->frame_payload_remaining;
    }

    if (contiguous > 0U)
    {
        slot->registration.on_data(slot->registration.context,
                                   &client->rx_buffer[tail_index],
                                   contiguous);
        client->rx_tail = tail + (uint32_t)contiguous;
        client->frame_payload_remaining -= contiguous;
    }

    if (client->frame_payload_remaining == 0U)
    {
        slot->registration.on_end(slot->registration.context,
                                  ATCLIENT_RESULT_OK);
        client->parser_state = ATCLIENT_PARSER_LINE;
    }
}
