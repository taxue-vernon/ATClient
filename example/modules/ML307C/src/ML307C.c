/**
 * @file  ML307C.c
 * @brief ML307C 4G 模组驱动实现（协作式状态机）。公开接口的形参说明与示例见 ML307C.h。
 */

#include "ML307C.h"
#include <stdio.h>
#include <string.h>

#define COMMAND_MS  5000U
#define HTTP_MS     65000U
#define HEALTH_MS   30000U
#define CACHE_BYTES 4096U

/**
 * @brief 切换对外状态，状态确实变化时调用 on_state_changed。
 * @param m 传当前 ML307C 对象指针（内部调用时就是公开接口收到的 m）。
 * @param s 传要切换到的状态，如 ML307C_STATE_NETWORK_WAIT。
 * @return 无
 * @par 示例
 * @code
 * state(m, ML307C_STATE_NETWORK_WAIT);
 * @endcode
 */
static void state(ML307C *m, ML307C_State s)
{
    if (m->state == s)
        return;
    m->state = s;
    if (m->callbacks.on_state_changed)
        m->callbacks.on_state_changed(m->callback_context, s);
}

/**
 * @brief 跳过游标处的连续空格。
 * @param p 传字符串游标变量的地址（&p）；返回时 p 指向第一个非空格字符。
 * @return 无
 * @par 示例
 * @code
 * const char *p = "  12";
 * spaces(&p);   // p 指向 "12"
 * @endcode
 */
static void spaces(const char **p)
{
    while (**p == ' ')
        ++*p;
}

/**
 * @brief 解析一个无符号十进制整数（前后空格会被跳过），溢出 uint32_t 视为失败。
 * @param p 传字符串游标变量的地址（&p）；成功时 p 前移到数字之后。
 * @param v 输出：传一个 uint32_t 变量的地址，用于接收解析结果。
 * @return true 解析成功；false 不是数字或溢出。
 * @par 示例
 * @code
 * const char *p = "+CSQ: 20,99" + 5;
 * uint32_t rssi;
 * if (number(&p, &rssi)) {
 *     // rssi == 20，p 指向 ",99"
 * }
 * @endcode
 */
static bool number(const char **p, uint32_t *v)
{
    uint32_t n = 0;
    spaces(p);
    if (**p < '0' || **p > '9')
        return false;
    do {
        uint32_t d = (uint32_t)(**p - '0');
        if (n > (UINT32_MAX - d) / 10U)
            return false;
        n = n * 10U + d;
        ++*p;
    } while (**p >= '0' && **p <= '9');
    spaces(p);
    *v = n;
    return true;
}

/**
 * @brief 消费一个逗号（允许两侧空格）。
 * @param p 传字符串游标变量的地址（&p）；成功时 p 前移到逗号之后。
 * @return true 遇到逗号；false 当前字符不是逗号。
 * @par 示例
 * @code
 * if (number(&p, &a) && comma(&p) && number(&p, &b)) {
 *     // 解析 "a,b"
 * }
 * @endcode
 */
static bool comma(const char **p)
{
    spaces(p);
    if (**p != ',')
        return false;
    ++*p;
    spaces(p);
    return true;
}

/**
 * @brief 解析一个双引号括起的字符串字段（不支持转义，拒绝控制字符）。
 * @param p   传字符串游标变量的地址（&p），p 应指向左引号；成功时前移到右引号之后。
 * @param out 输出：传接收字段内容的字符数组，结果以 '\0' 结尾。
 * @param cap 传 out 的容量（含结尾 '\0'），一般用 sizeof(out)。
 * @return true 解析成功；false 格式错误或超长。
 * @par 示例
 * @code
 * char event[16];
 * const char *p = "\"recv\",1";
 * if (quoted(&p, event, sizeof(event))) {
 *     // event == "recv"
 * }
 * @endcode
 */
static bool quoted(const char **p, char *out, size_t cap)
{
    size_t n = 0;
    spaces(p);
    if (*(*p)++ != '"')
        return false;
    while (**p && **p != '"') {
        if ((unsigned char)**p < 32U || n + 1 >= cap)
            return false;
        out[n++] = *(*p)++;
    }
    if (**p != '"')
        return false;
    ++*p;
    spaces(p);
    out[n] = 0;
    return true;
}

/**
 * @brief 复制一个非空字符串到定长缓冲区。
 * @param out 输出：传目标字符数组，如 m->status.model。
 * @param cap 传目标数组容量，一般用 sizeof(m->status.model)。
 * @param s   传要复制的非空字符串。
 * @return true 成功；false 源为空串或放不下。
 * @par 示例
 * @code
 * (void)copy_field(m->status.model, sizeof(m->status.model), "ML307C");
 * @endcode
 */
static bool copy_field(char *out, size_t cap, const char *s)
{
    size_t n = strlen(s);
    if (!n || n >= cap)
        return false;
    memcpy(out, s, n + 1);
    return true;
}

/**
 * @brief 检查字符串能否安全嵌入 AT 命令的引号参数中（非空、可打印、无引号和反斜杠）。
 * @param s   传要检查的字符串，可以是 NULL（视为不安全）。
 * @param max 传允许的最大长度（字节），如路径传 256。
 * @return true 安全；false 为空、超长或含非法字符。
 * @par 示例
 * @code
 * if (!safe_field(path, 256)) {
 *     return ML307C_RESULT_INVALID_ARGUMENT;
 * }
 * @endcode
 */
static bool safe_field(const char *s, size_t max)
{
    size_t i;
    if (!s || !s[0])
        return false;
    for (i = 0; s[i]; ++i)
        if (i >= max || (unsigned char)s[i] < 32U || (unsigned char)s[i] > 126U || s[i] == '"' ||
            s[i] == '\\')
            return false;
    return true;
}

/**
 * @brief 判断是否已注册到网络（CEREG 状态 1=本地 或 5=漫游）。
 * @param m 传当前 ML307C 对象指针（内部调用时就是公开接口收到的 m）。
 * @return true 已注册；false 未注册。
 * @par 示例
 * @code
 * if (!registered(m)) {
 *     retry(m, ML307C_RESULT_NOT_READY, ML_N_SIM);
 * }
 * @endcode
 */
static bool registered(const ML307C *m)
{
    return m->status.registration == 1U || m->status.registration == 5U;
}

/**
 * @brief 判断当前 HTTP 配置是否为 https:// 服务器。
 * @param m 传当前 ML307C 对象指针（内部调用时就是公开接口收到的 m）。
 * @return true 已配置 HTTPS；false 未配置或为 HTTP。
 * @par 示例
 * @code
 * m->stage = https_configured(m) ? ML_H_SSL : ML_H_CACHE;
 * @endcode
 */
static bool https_configured(const ML307C *m)
{
    return m->http_config.host && !strncmp(m->http_config.host, "https://", 8);
}

/**
 * @brief 把 ATClient 结果码映射为 ML307C 结果码。
 * @param r 传 ATClient 返回的结果码（通常是 m->at_result）。
 * @return 对应的 ML307C_Result（超时/解析错误/其余视为 AT 错误）。
 * @par 示例
 * @code
 * ML307C_Result r = map_result(m->at_result);
 * @endcode
 */
static ML307C_Result map_result(ATClient_Result r)
{
    if (r == ATCLIENT_RESULT_OK)
        return ML307C_RESULT_OK;
    if (r == ATCLIENT_RESULT_TIMEOUT)
        return ML307C_RESULT_TIMEOUT;
    if (r == ATCLIENT_RESULT_PROTOCOL_ERROR || r == ATCLIENT_RESULT_LINE_TOO_LONG ||
        r == ATCLIENT_RESULT_RX_OVERFLOW)
        return ML307C_RESULT_PARSE_ERROR;
    return ML307C_RESULT_AT_ERROR;
}

/**
 * @brief 标记网络丢失：清空 IP 与 TLS 状态，必要时中止正在进行的短信事务。
 * @param m 传当前 ML307C 对象指针（内部调用时就是公开接口收到的 m）。
 * @return 无
 * @par 示例
 * @code
 * if (!registered(m) && m->status.data_active) {
 *     lost(m);
 * }
 * @endcode
 */
static void lost(ML307C *m)
{
    m->status.data_active = false;
    m->status.tls_ready = false;
    m->status.ipv4[0] = m->status.ipv6[0] = 0;
    m->network_lost = true;
    if (!m->sms_channel_uncertain)
        state(m, ML307C_STATE_NETWORK_WAIT);
    /* A disconnected modem cannot finish an in-flight SMS transaction.
     * Wake ATClient so the main loop reports failure without waiting 120 s. */
    if (m->status.sms_busy && m->pending)
        ATClient_OnTransportError(m->at);
}

/**
 * @brief URC 统一处理回调：+MATREADY、+CEREG、+MIPCALL、+MHTTPURC。
 * @details 解析失败会累加 urc_error_count / parse_error_count，并把当前事务标记为解析错误。
 * @param ctx 由 ATClient 传入：注册时给的 ML307C 对象指针。
 * @param s   由 ATClient 传入：以 '\0' 结尾的 URC 行，如 "+CEREG: 1"。
 * @param len 由 ATClient 传入：行长度（未使用）。
 * @return 无
 * @par 示例
 * @code
 * (void)ATClient_RegisterUrc(at, "+CEREG:", urc, m);
 * @endcode
 */
static void urc(void *ctx, const char *s, size_t len)
{
    ML307C *m = ctx;
    const char *p;
    uint32_t a, b, c, d, e;
    bool ok = false;
    (void)len;
    if (!strcmp(s, "+MATREADY")) {
        m->status.matready_seen = true;
        /* During first probe MATREADY is expected; afterwards it invalidates
         * all server and PDP handles. In-flight SMS is aborted by lost(). */
        if (m->startup_notified || m->status.http_busy) {
            m->reboot = true;
            lost(m);
        }
        return;
    }
    if (!strncmp(s, "+CEREG:", 7)) {
        p = s + 7;
        if (number(&p, &a)) {
            b = a;
            if (*p == ',') {
                ++p;
                spaces(&p);
                /* Query: <n>,<stat>; notification: <stat>,"tac",... */
                if (*p != '"' && !number(&p, &b))
                    goto malformed;
            }
            if (b <= 10 && (!*p || *p == ',' || *p == '"')) {
                m->status.registration = (uint8_t)b;
                if (m->stage == ML_N_REG && m->pending)
                    m->value_valid = true;
                if (!registered(m) && m->status.data_active)
                    lost(m);
                return;
            }
        }
    } else if (!strncmp(s, "+MIPCALL:", 9)) {
        char ip1[46] = {0}, ip2[46] = {0}, v4[16] = {0}, v6[46] = {0};
        p = s + 9;
        if (!number(&p, &a) || !comma(&p) || !number(&p, &b) || b > 1)
            goto malformed;
        if (a != 1)
            return; /* Only CID 1 belongs to this driver. */
        if (b == 1) {
            if (!comma(&p) || !quoted(&p, ip1, sizeof(ip1)))
                goto malformed;
            if (*p && (!comma(&p) || !quoted(&p, ip2, sizeof(ip2))))
                goto malformed;
            if (*p)
                goto malformed;
            const char *ips[2] = {ip1, ip2};
            for (unsigned i = 0; i < 2; ++i) {
                if (!ips[i][0])
                    continue;
                if (strchr(ips[i], ':')) {
                    if (!copy_field(v6, sizeof(v6), ips[i]))
                        goto malformed;
                } else if (!strchr(ips[i], '.') || !copy_field(v4, sizeof(v4), ips[i]))
                    goto malformed;
            }
            if (!v4[0] && !v6[0])
                goto malformed;
        } else if (*p)
            goto malformed;
        memcpy(m->status.ipv4, v4, sizeof(v4));
        memcpy(m->status.ipv6, v6, sizeof(v6));
        m->status.data_active = (b == 1);
        m->pdp_notice = true;
        if (m->stage == ML_N_PDP && m->pending)
            m->value_valid = true;
        if (!b && m->startup_notified)
            lost(m);
        return;
    } else if (!strncmp(s, "+MHTTPURC:", 10)) {
        char event[16];
        p = s + 10;
        if (!quoted(&p, event, sizeof(event)) || !comma(&p) || !number(&p, &a))
            goto malformed;
        if (!m->status.http_busy || (int)a != m->http_id)
            return;
        if (!strcmp(event, "recv")) {
            if (!comma(&p) || !number(&p, &b) || !comma(&p) || !number(&p, &c) || !comma(&p) ||
                !number(&p, &d) || *p || b < 100 || b > 599 || c > CACHE_BYTES || d > CACHE_BYTES ||
                c + d > CACHE_BYTES)
                goto malformed;
            if (m->stage != ML_H_REQUEST && m->stage != ML_H_WAIT)
                return;
            if (m->response_seen)
                return;
            m->response_seen = true;
            m->status.http_status = (uint16_t)b;
            m->remaining[0] = c;
            m->remaining[1] = d;
            if (m->callbacks.on_http_response)
                m->callbacks.on_http_response(m->callback_context, (uint16_t)b, c, d);
            return;
        }
        if (!strcmp(event, "err")) {
            ok = comma(&p) && number(&p, &e) && !*p;
            if (!ok)
                goto malformed;
            m->status.http_error = e;
            m->http_result = ML307C_RESULT_HTTP_ERROR;
            return;
        }
        return;
    } else
        return;
malformed:
    ++m->status.urc_error_count;
    ++m->status.parse_error_count;
    if (m->pending)
        m->bad_value = true;
    if (m->status.http_busy)
        m->http_result = ML307C_RESULT_PARSE_ERROR;
}

/**
 * @brief 当前命令的响应行回调，按命令阶段解析 CCLK、MSSLLIST、ATI、CGMR、CGSN、CMGS、
 *        CPIN、CFUN、CSQ、MHTTPCREATE 等返回值。
 * @param ctx 由 ATClient 传入：请求的 callback_context，即 ML307C 对象指针。
 * @param s   由 ATClient 传入：以 '\0' 结尾的响应行，如 "+CSQ: 20,99"。
 * @param len 由 ATClient 传入：行长度（未使用）。
 * @return 无
 * @par 示例
 * @code
 * ATClient_Request req = {0};
 * req.on_line = on_line;
 * req.callback_context = m;
 * @endcode
 */
static void on_line(void *ctx, const char *s, size_t len)
{
    ML307C *m = ctx;
    const char *p = s;
    uint32_t a, b;
    bool ok = false;
    (void)len;
    switch (m->stage) {
    case ML_T_CLOCK:
        if (strncmp(s, "+CCLK: \"", 8))
            return;
        p = s + 8;
        /* The modem starts with an invalid clock. Only network-sourced
         * dates from 2024 onward are useful for certificate expiry checks. */
        ok = strlen(p) == 21 && p[0] >= '0' && p[0] <= '6' && p[1] >= '0' && p[1] <= '9' &&
             (p[0] > '2' || (p[0] == '2' && p[1] >= '4')) && p[2] == '/' && p[5] == '/' &&
             p[8] == ',' && p[11] == ':' && p[14] == ':' && (p[17] == '+' || p[17] == '-') &&
             p[20] == '"';
        for (size_t i = 3; ok && i < 20; ++i)
            if (i != 5 && i != 8 && i != 11 && i != 14 && i != 17 && (p[i] < '0' || p[i] > '9'))
                ok = false;
        if (ok) {
            unsigned year = (unsigned)(p[0] - '0') * 10U + (unsigned)(p[1] - '0');
            unsigned month = (unsigned)(p[3] - '0') * 10U + (unsigned)(p[4] - '0');
            unsigned day = (unsigned)(p[6] - '0') * 10U + (unsigned)(p[7] - '0');
            unsigned hour = (unsigned)(p[9] - '0') * 10U + (unsigned)(p[10] - '0');
            unsigned minute = (unsigned)(p[12] - '0') * 10U + (unsigned)(p[13] - '0');
            unsigned second = (unsigned)(p[15] - '0') * 10U + (unsigned)(p[16] - '0');
            unsigned zone = (unsigned)(p[18] - '0') * 10U + (unsigned)(p[19] - '0');
            static const uint8_t days[] = {0, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
            unsigned max_day = month >= 1U && month <= 12U ? days[month] : 0U;
            if (month == 2U && year % 4U == 0U)
                max_day = 29U;
            ok = day >= 1U && day <= max_day && hour < 24U && minute < 60U && second < 60U &&
                 zone <= 56U;
        }
        break;
    case ML_T_LIST:
        if (strncmp(s, "+MSSLLIST:", 10))
            return;
        {
            char name[64];
            p = s + 10;
            ok = quoted(&p, name, sizeof(name)) && comma(&p) && number(&p, &a) && !*p;
            if (ok && !strcmp(name, m->http_config.ca_name) && a == m->http_config.ca_length)
                m->tls_cert_present = true;
        }
        break;
    case ML_N_MODEL:
        if (!strncmp(s, "Model:", 6)) {
            p = s + 6;
            spaces(&p);
        } else if (strncmp(s, "ML307", 5))
            return;
        ok = copy_field(m->status.model, sizeof(m->status.model), p);
        break;
    case ML_N_FW:
        if (!strncmp(s, "+CGMR:", 6)) {
            p = s + 6;
            spaces(&p);
        }
        ok = copy_field(m->status.firmware, sizeof(m->status.firmware), p);
        break;
    case ML_N_IMEI:
        if (!strncmp(s, "+CGSN:", 6)) {
            p = s + 6;
            spaces(&p);
        }
        ok = strlen(p) == 15;
        for (size_t i = 0; ok && i < 15; ++i)
            if (p[i] < '0' || p[i] > '9')
                ok = false;
        if (ok)
            memcpy(m->status.imei, p, 16);
        break;
    case ML_S_SEND:
        if (strncmp(s, "+CMGS:", 6))
            return;
        p = s + 6;
        ok = number(&p, &a) && !*p && a <= 255U;
        if (ok)
            m->status.sms_reference = a;
        break;
    case ML_N_SIM:
        if (strncmp(s, "+CPIN:", 6))
            return;
        p = s + 6;
        spaces(&p);
        m->status.sim_ready = !strcmp(p, "READY");
        ok = *p != 0;
        break;
    case ML_N_CFUN:
        if (strncmp(s, "+CFUN:", 6))
            return;
        p = s + 6;
        ok = number(&p, &a) && !*p && a <= 127;
        if (ok)
            m->status.cfun = (uint8_t)a;
        break;
    case ML_N_CSQ:
        if (strncmp(s, "+CSQ:", 5))
            return;
        p = s + 5;
        ok = number(&p, &a) && comma(&p) && number(&p, &b) && !*p && (a <= 31 || a == 99) &&
             (b <= 7 || b == 99);
        if (ok) {
            m->status.rssi = (uint8_t)a;
            m->status.ber = (uint8_t)b;
            m->status.rssi_known = a != 99;
            m->status.rssi_dbm = a == 99 ? 0 : (int16_t)(2 * (int)a - 113);
        }
        break;
    case ML_H_CREATE:
        if (strncmp(s, "+MHTTPCREATE:", 13))
            return;
        p = s + 13;
        ok = number(&p, &a) && !*p && a <= 3;
        if (ok)
            m->http_id = (int)a;
        break;
    default:
        return;
    }
    if (ok)
        m->value_valid = true;
    else {
        m->bad_value = true;
        ++m->status.parse_error_count;
    }
}

/**
 * @brief 当前命令完成回调：记录结果，交给下一次 ML307C_Process() 的 handle_done() 处理。
 * @param ctx 由 ATClient 传入：请求的 callback_context，即 ML307C 对象指针。
 * @param r   由 ATClient 传入：这条命令的完成结果。
 * @return 无
 * @par 示例
 * @code
 * req.on_complete = on_complete;
 * @endcode
 */
static void on_complete(void *ctx, ATClient_Result r)
{
    ML307C *m = ctx;
    m->pending = false;
    m->done = true;
    m->at_result = r;
}

/* Probe is called for each header byte by ATClient. Once the fourth comma
 * arrives, raw data_len bytes (including NUL/newlines) belong to this frame. */
/**
 * @brief +MHTTPREAD 定长帧头探测：等到第 4 个逗号后解析出负载长度。
 * @param ctx    由 ATClient 传入：注册时的 context（未使用）。
 * @param h      由 ATClient 传入：当前已收到的帧头字节。
 * @param n      由 ATClient 传入：h 的字节数。
 * @param length 输出：返回 READY 时写入负载字节数（≤ATCLIENT_FRAME_PAYLOAD_MAX）。
 * @return NO_MATCH / NEED_MORE / READY / ERROR。
 * @par 示例
 * @code
 * // "+MHTTPREAD: 0,1,100,256," 之后紧跟 256 字节正文
 * ATClient_LengthFrameRegistration f = {"+MHTTPREAD:", probe, frame_begin,
 *                                       frame_data, frame_end, m};
 * @endcode
 */
static ATClient_FrameProbeResult probe(void *ctx, const uint8_t *h, size_t n, size_t *length)
{
    static const char prefix[] = "+MHTTPREAD:";
    const size_t plen = sizeof(prefix) - 1;
    char text[96];
    const char *p;
    uint32_t a, b, c, d;
    unsigned commas = 0;
    (void)ctx;
    if (memcmp(h, prefix, n < plen ? n : plen))
        return ATCLIENT_FRAME_NO_MATCH;
    if (n < plen)
        return ATCLIENT_FRAME_NEED_MORE;
    for (size_t i = plen; i < n; ++i)
        if (h[i] == ',')
            ++commas;
    if (commas < 4)
        return ATCLIENT_FRAME_NEED_MORE;
    if (n >= sizeof(text))
        return ATCLIENT_FRAME_ERROR;
    memcpy(text, h, n);
    text[n] = 0;
    p = text + plen;
    if (!number(&p, &a) || !comma(&p) || !number(&p, &b) || !comma(&p) || !number(&p, &c) ||
        !comma(&p) || !number(&p, &d) || !comma(&p) || *p || a > 3 || b > 1 ||
        d > ATCLIENT_FRAME_PAYLOAD_MAX)
        return ATCLIENT_FRAME_ERROR;
    *length = d;
    return ATCLIENT_FRAME_READY;
}

/**
 * @brief +MHTTPREAD 帧开始：解析 ID、类型、剩余字节并校验是否属于当前读取请求。
 * @param ctx    由 ATClient 传入：注册时给的 ML307C 对象指针。
 * @param h      由 ATClient 传入：完整帧头，如 "+MHTTPREAD: 0,1,100,256,"。
 * @param n      由 ATClient 传入：h 的字节数。
 * @param length 由 ATClient 传入：本帧负载字节数。
 * @return 无
 * @par 示例
 * @code
 * // 由 ATClient 在 probe() 返回 READY 后自动调用
 * slot->registration.on_begin(slot->registration.context, header, header_length, length);
 * @endcode
 */
static void frame_begin(void *ctx, const uint8_t *h, size_t n, size_t length)
{
    ML307C *m = ctx;
    char text[96];
    const char *p;
    uint32_t a = 0, b = 0, c = 0, d = 0;
    memcpy(text, h, n);
    text[n] = 0;
    p = text + 11;
    (void)number(&p, &a);
    (void)comma(&p);
    (void)number(&p, &b);
    (void)comma(&p);
    (void)number(&p, &c);
    (void)comma(&p);
    (void)number(&p, &d);
    m->frame_seen = true;
    m->frame_size = length;
    m->frame_unread = c;
    m->frame_received = 0;
    m->frame_complete = false;
    m->frame_valid = m->pending && m->stage == ML_H_READ && (int)a == m->http_id &&
                     b == m->read_type && length > 0 && length <= m->read_size &&
                     length <= m->remaining[m->read_type] &&
                     c == m->remaining[m->read_type] - length;
    if (!m->frame_valid)
        m->http_result = ML307C_RESULT_PARSE_ERROR;
}

/**
 * @brief +MHTTPREAD 帧负载数据：校验通过时转发给 on_http_data，并统计正文字节数。
 * @param ctx 由 ATClient 传入：注册时给的 ML307C 对象指针。
 * @param d   由 ATClient 传入：本次收到的负载片段。
 * @param n   由 ATClient 传入：d 的字节数。
 * @return 无
 * @par 示例
 * @code
 * // 由 ATClient 分片调用
 * slot->registration.on_data(slot->registration.context, data, length);
 * @endcode
 */
static void frame_data(void *ctx, const uint8_t *d, size_t n)
{
    ML307C *m = ctx;
    m->frame_received += n;
    if (!m->frame_valid || m->http_result != ML307C_RESULT_OK || m->network_lost)
        return;
    if (m->read_type == 1)
        m->status.http_received_bytes += (uint32_t)n;
    if (m->callbacks.on_http_data)
        m->callbacks.on_http_data(m->callback_context, (ML307C_HttpDataType)m->read_type, d, n);
}

/**
 * @brief +MHTTPREAD 帧结束：记录负载是否完整收齐。
 * @param ctx 由 ATClient 传入：注册时给的 ML307C 对象指针。
 * @param r   由 ATClient 传入：ATCLIENT_RESULT_OK 表示正常结束。
 * @return 无
 * @par 示例
 * @code
 * slot->registration.on_end(slot->registration.context, ATCLIENT_RESULT_OK);
 * @endcode
 */
static void frame_end(void *ctx, ATClient_Result r)
{
    ML307C *m = ctx;
    m->frame_complete = (r == ATCLIENT_RESULT_OK && m->frame_received == m->frame_size);
}

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
bool ML307C_Init(ML307C *m, ATClient *at, const ML307C_Callbacks *cb, void *ctx)
{
    ATClient_LengthFrameRegistration f = {"+MHTTPREAD:", probe,     frame_begin,
                                          frame_data,    frame_end, m};
    if (!m || !at)
        return false;
    memset(m, 0, sizeof(*m));
    m->at = at;
    if (cb)
        m->callbacks = *cb;
    m->callback_context = ctx;
    m->http_id = -1;
    return ATClient_RegisterUrc(at, "+MATREADY", urc, m) == ATCLIENT_RESULT_OK &&
           ATClient_RegisterUrc(at, "+CEREG:", urc, m) == ATCLIENT_RESULT_OK &&
           ATClient_RegisterUrc(at, "+MIPCALL:", urc, m) == ATCLIENT_RESULT_OK &&
           ATClient_RegisterUrc(at, "+MHTTPURC:", urc, m) == ATCLIENT_RESULT_OK &&
           ATClient_RegisterLengthFrame(at, &f) == ATCLIENT_RESULT_OK;
}

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
ML307C_Result ML307C_Start(ML307C *m)
{
    if (!m || !m->at)
        return ML307C_RESULT_INVALID_ARGUMENT;
    if (m->started || m->pending)
        return ML307C_RESULT_BUSY;
    m->started = true;
    m->stage = ML_N_AT;
    state(m, ML307C_STATE_PROBING);
    return ML307C_RESULT_OK;
}

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
ML307C_State ML307C_GetState(const ML307C *m)
{
    return m ? m->state : ML307C_STATE_OFFLINE;
}

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
const ML307C_Status *ML307C_GetStatus(const ML307C *m)
{
    return m ? &m->status : NULL;
}

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
void ML307C_RequestIdleSleep(ML307C *m, bool enabled)
{
    if (m)
        m->sleep_requested = enabled;
}

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
bool ML307C_SleepReady(const ML307C *m)
{
    return m && m->sleep_requested && m->sleep_configured && !m->sleeping && !m->pending &&
           m->stage == ML_N_IDLE && !m->status.http_busy && !m->status.sms_busy &&
           !m->network_lost && !m->reboot && m->state == ML307C_STATE_NETWORK_READY &&
           m->status.data_active && (!https_configured(m) || m->status.tls_ready) &&
           ATClient_IsIdle(m->at);
}

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
bool ML307C_EnterSleep(ML307C *m)
{
    if (!ML307C_SleepReady(m))
        return false;
    m->sleeping = true;
    return true;
}

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
void ML307C_Wake(ML307C *m)
{
    if (!m || !m->sleeping)
        return;
    m->sleeping = false;
    m->sleep_requested = false;
    if (m->reboot)
        m->sleep_configured = false;
    m->retry_wait = false;
    m->network_lost = false;
    m->reboot = false;
    m->status.responsive = false;
    m->status.data_active = false;
    m->status.tls_ready = false;
    m->status.registration = 0;
    m->status.ipv4[0] = m->status.ipv6[0] = 0;
    m->stage = ML_N_AT;
    state(m, ML307C_STATE_PROBING);
}

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
bool ML307C_HttpConfigure(ML307C *m, const ML307C_HttpConfig *cfg)
{
    size_t scheme;
    if (!m || !cfg || m->status.http_busy || !safe_field(cfg->host, 192) ||
        cfg->read_chunk_size > 1024)
        return false;
    if (!strncmp(cfg->host, "https://", 8)) {
        scheme = 8;
        if (!cfg->ca_pem || !cfg->ca_length || cfg->ca_length > 8192 ||
            !safe_field(cfg->ca_name, 63))
            return false;
    } else if (!strncmp(cfg->host, "http://", 7))
        scheme = 7;
    else
        return false;
    if (!cfg->host[scheme])
        return false;
    /* Host is an authority only; path belongs to each request. */
    for (const char *p = cfg->host + scheme; *p; ++p)
        if (*p == '/' || *p == '?' || *p == '#' || *p == ' ' || *p == '@')
            return false;
    m->http_config = *cfg;
    m->status.tls_ready = false;
    if (!m->http_config.read_chunk_size)
        m->http_config.read_chunk_size = 256;
    return true;
}

/**
 * @brief GET/POST 公共入口：校验路径与就绪条件，初始化 HTTP 事务并进入 ML_H_CREATE 阶段。
 * @param m      传当前 ML307C 对象指针（内部调用时就是公开接口收到的 m）。
 * @param path   传 HttpGet/HttpPost 收到的请求路径（以 '/' 开头）。
 * @param method 传 1 表示 GET，传 2 表示 POST（即 AT+MHTTPREQUEST 的 method 参数）。
 * @return ML307C_RESULT_OK / BUSY / NOT_READY / INVALID_ARGUMENT。
 * @par 示例
 * @code
 * ML307C_Result r = http_start(m, "/api/v1/send", 2);
 * @endcode
 */
static ML307C_Result http_start(ML307C *m, const char *path, unsigned method)
{
    if (!m || !safe_field(path, 256) || path[0] != '/')
        return ML307C_RESULT_INVALID_ARGUMENT;
    if (https_configured(m) && !m->status.tls_ready)
        return ML307C_RESULT_NOT_READY;
    if (m->sleeping)
        return ML307C_RESULT_NOT_READY;
    if (m->state == ML307C_STATE_PROBING)
        return ML307C_RESULT_NOT_READY;
    if (m->status.http_busy || m->pending || m->stage != ML_N_IDLE)
        return ML307C_RESULT_BUSY;
    /* A failed delete leaves ownership ambiguous. Require a modem restart
     * before allocating another instance instead of leaking all four IDs. */
    if (m->status.cleanup_result != ML307C_RESULT_OK)
        return ML307C_RESULT_NOT_READY;
    if (!m->http_config.host || m->state != ML307C_STATE_NETWORK_READY || !m->status.data_active ||
        !registered(m))
        return ML307C_RESULT_NOT_READY;
    m->path = path;
    m->method = method;
    m->http_id = -1;
    m->response_seen = false;
    m->http_result = ML307C_RESULT_OK;
    m->status.http_busy = true;
    m->status.http_status = 0;
    m->status.http_error = 0;
    m->status.http_received_bytes = 0;
    m->status.cleanup_result = ML307C_RESULT_OK;
    ++m->status.http_request_count;
    m->stage = ML_H_CREATE;
    return ML307C_RESULT_OK;
}

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
ML307C_Result ML307C_HttpGet(ML307C *m, const char *path)
{
    return http_start(m, path, 1);
}

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
                              size_t length)
{
    ML307C_Result r;
    if (!safe_field(type, 96) || !body || !length || length > 4096)
        return ML307C_RESULT_INVALID_ARGUMENT;
    r = http_start(m, path, 2);
    if (r != ML307C_RESULT_OK)
        return r;
    (void)snprintf(m->header, sizeof(m->header), "Content-Type: %s\r\n", type);
    m->body = body;
    m->body_length = length;
    return r;
}

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
ML307C_Result ML307C_SendSms(ML307C *m, const char *number, const char *text)
{
    size_t n = 0, length = 0;
    if (!m || !number || !text)
        return ML307C_RESULT_INVALID_ARGUMENT;
    if (number[0] == '+')
        n = 1;
    if (number[n] < '0' || number[n] > '9')
        return ML307C_RESULT_INVALID_ARGUMENT;
    for (; number[n]; ++n)
        if (n >= 20U || number[n] < '0' || number[n] > '9')
            return ML307C_RESULT_INVALID_ARGUMENT;
    for (; text[length]; ++length)
        if (length >= 160U || (unsigned char)text[length] < 32U ||
            (unsigned char)text[length] > 126U)
            return ML307C_RESULT_INVALID_ARGUMENT;
    if (!length)
        return ML307C_RESULT_INVALID_ARGUMENT;
    if (m->sleeping)
        return ML307C_RESULT_NOT_READY;
    if (m->status.sms_busy || m->status.http_busy || m->pending || m->stage != ML_N_IDLE)
        return ML307C_RESULT_BUSY;
    if (m->state != ML307C_STATE_NETWORK_READY || !m->status.sim_ready || !registered(m))
        return ML307C_RESULT_NOT_READY;
    memcpy(m->sms_number, number, n + 1);
    memcpy(m->sms_payload, text, length);
    m->sms_payload[length] = 0x1AU;
    m->sms_payload_length = length + 1U;
    m->status.sms_reference = 0;
    m->status.sms_busy = true;
    m->stage = ML_S_MODE;
    return ML307C_RESULT_OK;
}

/**
 * @brief ATClient 命令构造回调：按当前阶段生成对应的 AT 命令文本。
 * @param ctx    由 ATClient 传入：command_context，即 ML307C 对象指针。
 * @param dst    由 ATClient 传入：命令写入的缓冲区。
 * @param cap    由 ATClient 传入：dst 的容量。
 * @param length 输出：写入命令长度。
 * @return true 生成成功；false 阶段无对应命令或缓冲区不足。
 * @par 示例
 * @code
 * ATClient_Request req = {0};
 * req.build_command = build;
 * req.command_context = m;
 * @endcode
 */
static bool build(void *ctx, char *dst, size_t cap, size_t *length)
{
    ML307C *m = ctx;
    int n = -1;
    /* Bare AT+CGSN returns ML307C's alphanumeric serial, not its IMEI. */
    static const char *const commands[] = {
        "",         "AT",       "ATE0",   "AT+CMEE=2", "ATI",         "AT+CGMR",       "AT+CGSN=1",
        "AT+CPIN?", "AT+CFUN?", "AT+CSQ", "AT+CEREG?", "AT+MIPCALL?", "AT+MIPCALL=1,1"};
    if (m->stage >= ML_N_AT && m->stage <= ML_N_ACTIVATE)
        n = snprintf(dst, cap, "%s", commands[m->stage]);
    else
        switch (m->stage) {
        case ML_T_CLOCK:
            n = snprintf(dst, cap, "AT+CCLK?");
            break;
        case ML_T_LIST:
            n = snprintf(dst, cap, "AT+MSSLLIST=1");
            break;
        case ML_T_WRITE:
            n = snprintf(dst, cap, "AT+MSSLCERTWR=\"%s\",0,%lu", m->http_config.ca_name,
                         (unsigned long)m->http_config.ca_length);
            break;
        case ML_T_CERT:
            n = snprintf(dst, cap, "AT+MSSLCFG=\"cert\",1,\"%s\"", m->http_config.ca_name);
            break;
        case ML_T_AUTH:
            n = snprintf(dst, cap, "AT+MSSLCFG=\"auth\",1,1");
            break;
        case ML_T_STAMP:
            n = snprintf(dst, cap, "AT+MSSLCFG=\"ignorestamp\",1,0");
            break;
        case ML_T_VERIFY:
            n = snprintf(dst, cap, "AT+MSSLCFG=\"ignoreverify\",1,0");
            break;
        case ML_T_SNI:
            n = snprintf(dst, cap, "AT+MSSLCFG=\"sni\",1,1");
            break;
        case ML_H_CREATE:
            n = snprintf(dst, cap, "AT+MHTTPCREATE=\"%s\"", m->http_config.host);
            break;
        case ML_H_SSL:
            n = snprintf(dst, cap, "AT+MHTTPCFG=\"ssl\",%d,1,1", m->http_id);
            break;
        case ML_H_CACHE:
            n = snprintf(dst, cap, "AT+MHTTPCFG=\"cached\",%d,1,4096", m->http_id);
            break;
        case ML_H_ENCODING:
            n = snprintf(dst, cap, "AT+MHTTPCFG=\"encoding\",%d,0,0", m->http_id);
            break;
        case ML_H_TIMEOUT:
            n = snprintf(dst, cap, "AT+MHTTPCFG=\"timeout\",%d,30,30,10", m->http_id);
            break;
        case ML_H_HEADER:
            n = snprintf(dst, cap, "AT+MHTTPHEADER=%d,0,%lu", m->http_id,
                         (unsigned long)strlen(m->header));
            break;
        case ML_H_BODY:
            n = snprintf(dst, cap, "AT+MHTTPCONTENT=%d,0,%lu", m->http_id,
                         (unsigned long)m->body_length);
            break;
        case ML_H_REQUEST:
            n = snprintf(dst, cap, "AT+MHTTPREQUEST=%d,%u,0,\"%s\"", m->http_id, m->method,
                         m->path);
            break;
        case ML_H_READ:
            n = snprintf(dst, cap, "AT+MHTTPREAD=%d,%u,%lu", m->http_id, m->read_type,
                         (unsigned long)m->read_size);
            break;
        case ML_H_DELETE:
            n = snprintf(dst, cap, "AT+MHTTPDEL=%d", m->http_id);
            break;
        case ML_S_MODE:
            n = snprintf(dst, cap, "AT+CMGF=1");
            break;
        case ML_S_SEND:
            n = snprintf(dst, cap, "AT+CMGS=\"%s\"", m->sms_number);
            break;
        case ML_P_SLEEP:
            n = snprintf(dst, cap, "AT+MLPMCFG=\"sleepmode\",2,0");
            break;
        default:
            break;
        }
    if (n <= 0 || (size_t)n >= cap)
        return false;
    *length = (size_t)n;
    return true;
}

/**
 * @brief 记录失败并进入 5 秒重试等待，到期后从 next 阶段继续。
 * @param m    传当前 ML307C 对象指针（内部调用时就是公开接口收到的 m）。
 * @param r    传本次失败的原因，如 ML307C_RESULT_NOT_READY。
 * @param next 传重试时从哪个阶段开始，如 ML_N_SIM。
 * @return 无
 * @par 示例
 * @code
 * retry(m, ML307C_RESULT_NOT_READY, ML_N_SIM);
 * @endcode
 */
static void retry(ML307C *m, ML307C_Result r, ML307C_Stage next)
{
    m->status.last_result = r;
    ++m->status.retry_count;
    m->retry_wait = true;
    m->due_ms = ATClient_NowMs(m->at) + 5000U;
    m->stage = next;
    state(m, ML307C_STATE_RETRY_WAIT);
}

/**
 * @brief 进入网络就绪状态；首次就绪时调用 on_startup_complete。
 * @param m 传当前 ML307C 对象指针（内部调用时就是公开接口收到的 m）。
 * @return 无
 * @par 示例
 * @code
 * if (m->pdp_notice && m->status.data_active && registered(m)) {
 *     ready(m);
 * }
 * @endcode
 */
static void ready(ML307C *m)
{
    m->network_lost = false;
    m->stage = ML_N_IDLE;
    m->health_ms = ATClient_NowMs(m->at);
    m->status.last_check_ms = m->health_ms;
    m->status.last_result = ML307C_RESULT_OK;
    state(m, ML307C_STATE_NETWORK_READY);
    if (!m->startup_notified) {
        m->startup_notified = true;
        if (m->callbacks.on_startup_complete)
            m->callbacks.on_startup_complete(m->callback_context, ML307C_RESULT_OK);
    }
}

/**
 * @brief 结束 HTTP 事务：清除忙标志、回到空闲阶段并调用 on_http_complete。
 * @param m 传当前 ML307C 对象指针；结果取自 m->http_result，调用前先设置好。
 * @return 无
 * @par 示例
 * @code
 * m->http_result = ML307C_RESULT_TIMEOUT;
 * http_finish(m);
 * @endcode
 */
static void http_finish(ML307C *m)
{
    ML307C_Result r = m->http_result;
    m->status.last_result = r;
    m->status.http_busy = false;
    m->http_id = -1;
    m->stage = ML_N_IDLE;
    /* Prevent immediate status traffic after a failed request; never replay
     * a POST whose server-side outcome is unknown. */
    m->health_ms = ATClient_NowMs(m->at);
    if (m->callbacks.on_http_complete)
        m->callbacks.on_http_complete(m->callback_context, r);
}

/**
 * @brief 结束短信事务并调用 on_sms_complete。
 * @details 若在 '>' 输入模式中超时/断网，标记通道不确定并进入 ERROR，等待模组重启 URC，
 *          避免后续 AT 命令被拼进短信正文。
 * @param m 传当前 ML307C 对象指针（内部调用时就是公开接口收到的 m）。
 * @param r 传短信最终结果，如 ML307C_RESULT_OK、ML307C_RESULT_NETWORK_LOST。
 * @return 无
 * @par 示例
 * @code
 * sms_finish(m, ML307C_RESULT_NETWORK_LOST);
 * @endcode
 */
static void sms_finish(ML307C *m, ML307C_Result r)
{
    /* A timed-out CMGS may still be in the module's '>' input mode. Sending
     * AT+CSQ now could append it to the SMS. Wait for a confirmed reset URC. */
    if (m->stage == ML_S_SEND &&
        (r == ML307C_RESULT_TIMEOUT || r == ML307C_RESULT_PARSE_ERROR ||
         (r == ML307C_RESULT_NETWORK_LOST && !m->reboot) ||
         (m->at_result == ATCLIENT_RESULT_TRANSPORT_ERROR && !m->reboot))) {
        m->sms_channel_uncertain = true;
        m->status.responsive = false;
        state(m, ML307C_STATE_ERROR);
    }
    m->status.last_result = r;
    m->status.sms_busy = false;
    m->stage = ML_N_IDLE;
    if (r != ML307C_RESULT_OK)
        m->status.sms_reference = 0;
    m->health_ms = ATClient_NowMs(m->at);
    if (m->callbacks.on_sms_complete)
        m->callbacks.on_sms_complete(m->callback_context, r, m->status.sms_reference);
}

/**
 * @brief 选择下一次 AT+MHTTPREAD 读取的类型（先头后正文）和块大小；都读完则进入删除阶段。
 * @param m 传当前 ML307C 对象指针（内部调用时就是公开接口收到的 m）。
 * @return 无
 * @par 示例
 * @code
 * if (m->response_seen) {
 *     choose_read(m);
 * }
 * @endcode
 */
static void choose_read(ML307C *m)
{
    if (m->remaining[0])
        m->read_type = 0;
    else if (m->remaining[1])
        m->read_type = 1;
    else {
        m->stage = ML_H_DELETE;
        return;
    }
    m->read_size = m->remaining[m->read_type];
    if (m->read_size > m->http_config.read_chunk_size)
        m->read_size = m->http_config.read_chunk_size;
    m->stage = ML_H_READ;
}

/**
 * @brief 处理上一条 AT 命令的完成结果，并决定下一阶段（睡眠/短信/TLS/HTTP/联网）。
 * @param m 传当前 ML307C 对象指针（内部调用时就是公开接口收到的 m）。
 * @return 无
 * @par 示例
 * @code
 * if (m->done) {
 *     m->done = false;
 *     handle_done(m);
 * }
 * @endcode
 */
static void handle_done(ML307C *m)
{
    ML307C_Result r = map_result(m->at_result);
    bool needs_value = (m->stage >= ML_N_MODEL && m->stage <= ML_N_PDP) || m->stage == ML_T_CLOCK ||
                       m->stage == ML_H_CREATE || m->stage == ML_S_SEND;
    if (r == ML307C_RESULT_OK && (m->bad_value || (needs_value && !m->value_valid)))
        r = ML307C_RESULT_PARSE_ERROR;
    if (m->stage == ML_P_SLEEP) {
        m->status.last_result = r;
        if (r == ML307C_RESULT_OK)
            m->sleep_configured = true;
        else
            m->sleep_due_ms = ATClient_NowMs(m->at) + 5000U;
        m->stage = ML_N_IDLE;
        m->health_ms = ATClient_NowMs(m->at);
        return;
    }
    if (m->status.sms_busy) {
        if (m->network_lost)
            r = ML307C_RESULT_NETWORK_LOST;
        if (r != ML307C_RESULT_OK)
            sms_finish(m, r);
        else if (m->stage == ML_S_MODE)
            m->stage = ML_S_SEND;
        else
            sms_finish(m, ML307C_RESULT_OK);
        return;
    }
    if (m->stage >= ML_T_CLOCK && m->stage <= ML_T_SNI) {
        if (m->network_lost)
            r = ML307C_RESULT_NETWORK_LOST;
        if (r != ML307C_RESULT_OK) {
            m->status.tls_ready = false;
            m->status.last_result = r;
            m->tls_due_ms = ATClient_NowMs(m->at) + 5000U;
            if (m->stage == ML_T_WRITE &&
                (r == ML307C_RESULT_TIMEOUT || m->at_result == ATCLIENT_RESULT_TRANSPORT_ERROR)) {
                m->tls_channel_uncertain = true;
                state(m, ML307C_STATE_ERROR);
            }
            m->stage = ML_N_IDLE;
        } else if (m->stage == ML_T_LIST) {
            m->stage = m->tls_cert_present ? ML_T_CERT : ML_T_WRITE;
        } else if (m->stage == ML_T_SNI) {
            m->status.tls_ready = true;
            m->status.last_result = ML307C_RESULT_OK;
            m->stage = ML_N_IDLE;
            m->health_ms = ATClient_NowMs(m->at);
        } else
            m->stage = (ML307C_Stage)(m->stage + 1);
        return;
    }
    if (m->status.http_busy) {
        if (m->network_lost)
            m->http_result = ML307C_RESULT_NETWORK_LOST;
        if (m->stage == ML_H_DELETE) {
            m->status.cleanup_result = r;
            if (m->http_result == ML307C_RESULT_OK)
                m->http_result = r;
            http_finish(m);
            return;
        }
        if (m->http_result == ML307C_RESULT_OK && r != ML307C_RESULT_OK)
            m->http_result = r;
        if (m->http_result != ML307C_RESULT_OK)
            return; /* process schedules cleanup */
        switch (m->stage) {
        case ML_H_CREATE:
            m->stage = https_configured(m) ? ML_H_SSL : ML_H_CACHE;
            break;
        case ML_H_SSL:
            m->stage = ML_H_CACHE;
            break;
        case ML_H_CACHE:
            m->stage = ML_H_ENCODING;
            break;
        case ML_H_ENCODING:
            m->stage = ML_H_TIMEOUT;
            break;
        case ML_H_TIMEOUT:
            m->stage = m->method == 2 ? ML_H_HEADER : ML_H_REQUEST;
            break;
        case ML_H_HEADER:
            m->stage = ML_H_BODY;
            break;
        case ML_H_BODY:
            m->stage = ML_H_REQUEST;
            break;
        case ML_H_REQUEST:
            m->stage = ML_H_WAIT;
            m->wait_ms = ATClient_NowMs(m->at);
            break;
        case ML_H_READ:
            if (!m->frame_seen || !m->frame_valid || !m->frame_complete)
                m->http_result = ML307C_RESULT_PARSE_ERROR;
            else {
                m->remaining[m->read_type] = m->frame_unread;
                choose_read(m);
            }
            break;
        default:
            break;
        }
        return;
    }
    if (r != ML307C_RESULT_OK) {
        if (r == ML307C_RESULT_AT_ERROR || r == ML307C_RESULT_TIMEOUT)
            m->status.responsive = false;
        if (m->startup_notified)
            lost(m);
        retry(m, r, m->startup_notified ? ML_N_AT : m->stage);
        return;
    }
    m->status.responsive = true;
    switch (m->stage) {
    case ML_N_SIM:
        if (!m->status.sim_ready) {
            state(m, ML307C_STATE_SIM_WAIT);
            retry(m, ML307C_RESULT_NOT_READY, ML_N_SIM);
        } else
            m->stage = ML_N_CFUN;
        break;
    case ML_N_CFUN:
        if (m->status.cfun != 1)
            retry(m, ML307C_RESULT_NOT_READY, ML_N_SIM);
        else
            m->stage = ML_N_CSQ;
        break;
    case ML_N_REG:
        if (!registered(m)) {
            state(m, ML307C_STATE_NETWORK_WAIT);
            retry(m, ML307C_RESULT_NOT_READY, ML_N_SIM);
        } else
            m->stage = ML_N_PDP;
        break;
    case ML_N_PDP:
        if (m->status.data_active)
            ready(m);
        else {
            m->stage = ML_N_ACTIVATE;
            m->pdp_notice = false;
            state(m, ML307C_STATE_DATA_ACTIVATING);
        }
        break;
    case ML_N_ACTIVATE:
        m->stage = ML_N_WAIT;
        m->wait_ms = ATClient_NowMs(m->at);
        break;
    default:
        m->stage = (ML307C_Stage)(m->stage + 1);
        break;
    }
}

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
void ML307C_Process(ML307C *m)
{
    ATClient_Request req = {0};
    uint32_t now_ms;
    if (!m || !m->started || m->sleeping)
        return;
    now_ms = ATClient_NowMs(m->at);
    /* No response polling loops: pending transactions return immediately.
     * ATClient invokes completion/URC callbacks on subsequent main iterations. */
    if (m->pending)
        return;
    if (m->reboot) {
        m->done = false;
        if (m->status.http_busy) {
            m->http_result = ML307C_RESULT_NETWORK_LOST;
            http_finish(m);
        }
        if (m->status.sms_busy)
            sms_finish(m, ML307C_RESULT_NETWORK_LOST);
        m->reboot = false;
        m->sms_channel_uncertain = false;
        m->tls_channel_uncertain = false;
        m->sleep_configured = false;
        m->status.tls_ready = false;
        m->status.cleanup_result = ML307C_RESULT_OK;
        m->startup_notified = false;
        retry(m, ML307C_RESULT_NETWORK_LOST, ML_N_AT);
    }
    if (m->done) {
        m->done = false;
        handle_done(m);
    }
    if (m->sms_channel_uncertain || m->tls_channel_uncertain)
        return;
    if (m->retry_wait) {
        if ((int32_t)(now_ms - m->due_ms) < 0)
            return;
        m->retry_wait = false;
    }
    if (m->status.http_busy) {
        if (m->network_lost)
            m->http_result = ML307C_RESULT_NETWORK_LOST;
        if (m->http_result != ML307C_RESULT_OK && m->stage != ML_H_DELETE) {
            if (m->http_id < 0) {
                http_finish(m);
                return;
            }
            m->stage = ML_H_DELETE;
        }
        if (m->stage == ML_H_WAIT) {
            if (m->response_seen)
                choose_read(m);
            else if ((uint32_t)(now_ms - m->wait_ms) >= HTTP_MS) {
                m->http_result = ML307C_RESULT_TIMEOUT;
                m->stage = ML_H_DELETE;
            } else
                return;
        }
    } else if (m->status.sms_busy) {
        if (m->network_lost) {
            sms_finish(m, ML307C_RESULT_NETWORK_LOST);
        }
    } else if (m->stage == ML_N_IDLE) {
        if (m->network_lost) {
            m->network_lost = false;
            retry(m, ML307C_RESULT_NETWORK_LOST, ML_N_SIM);
            return;
        }
        if (https_configured(m) && !m->status.tls_ready && m->state == ML307C_STATE_NETWORK_READY &&
            m->status.data_active && (int32_t)(now_ms - m->tls_due_ms) >= 0) {
            m->stage = ML_T_CLOCK;
        } else {
            if (m->sleep_requested && m->state == ML307C_STATE_NETWORK_READY &&
                m->status.data_active && (!https_configured(m) || m->status.tls_ready)) {
                if (m->sleep_configured)
                    return;
                if ((int32_t)(now_ms - m->sleep_due_ms) >= 0)
                    m->stage = ML_P_SLEEP;
                else
                    return;
            }
            if (m->stage != ML_P_SLEEP) {
                if ((uint32_t)(now_ms - m->health_ms) < HEALTH_MS)
                    return;
                m->stage = ML_N_CSQ;
            }
        }
    } else if (m->stage == ML_N_WAIT) {
        if (m->pdp_notice && m->status.data_active && registered(m)) {
            ready(m);
            return;
        }
        if ((uint32_t)(now_ms - m->wait_ms) >= HTTP_MS)
            retry(m, ML307C_RESULT_TIMEOUT, ML_N_SIM);
        return;
    }
    if (m->stage == ML_N_IDLE || !ATClient_IsIdle(m->at))
        return;
    req.build_command = build;
    req.command_context = m;
    req.timeout_ms = COMMAND_MS;
    req.on_line = on_line;
    req.on_complete = on_complete;
    req.callback_context = m;
    if (m->stage == ML_H_HEADER || m->stage == ML_H_BODY || m->stage == ML_T_WRITE) {
        /* Longer than the explicitly configured 10 s modem input timeout:
         * don't inject a cleanup AT command while still in data-input mode. */
        req.timeout_ms = 15000U;
        req.prompt_byte = '>';
        if (m->stage == ML_H_HEADER) {
            req.payload = (const uint8_t *)m->header;
            req.payload_length = strlen(m->header);
        } else if (m->stage == ML_H_BODY) {
            req.payload = m->body;
            req.payload_length = m->body_length;
        } else {
            req.payload = m->http_config.ca_pem;
            req.payload_length = m->http_config.ca_length;
        }
    } else if (m->stage == ML_S_SEND) {
        /* ATClient waits for '>' and DMA-sends the exact text + Ctrl-Z bytes.
         * A long network wait is a deadline, never a blocking delay. */
        req.timeout_ms = 120000U;
        req.prompt_byte = '>';
        req.payload = m->sms_payload;
        req.payload_length = m->sms_payload_length;
    }
    m->value_valid = m->bad_value = m->frame_seen = m->frame_complete = false;
    if (ATClient_Submit(m->at, &req) == ATCLIENT_RESULT_OK)
        m->pending = true;
}
