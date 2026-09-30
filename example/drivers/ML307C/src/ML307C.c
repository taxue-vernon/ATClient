#include "ML307C.h"
#include <stdio.h>
#include <string.h>

#define COMMAND_MS 5000U
#define HTTP_MS 65000U
#define HEALTH_MS 30000U
#define CACHE_BYTES 4096U

static void state(ML307C *m, ML307C_State s)
{
    if (m->state == s) return;
    m->state=s;
    if (m->callbacks.on_state_changed) m->callbacks.on_state_changed(m->callback_context,s);
}
static void spaces(const char **p) { while (**p==' ') ++*p; }
static bool number(const char **p, uint32_t *v)
{
    uint32_t n=0; spaces(p);
    if (**p<'0'||**p>'9') return false;
    do {
        uint32_t d=(uint32_t)(**p-'0');
        if(n>(UINT32_MAX-d)/10U) return false;
        n=n*10U+d; ++*p;
    } while (**p>='0'&&**p<='9');
    spaces(p); *v=n; return true;
}
static bool comma(const char **p)
{ spaces(p); if(**p!=',') return false; ++*p; spaces(p); return true; }
static bool quoted(const char **p, char *out, size_t cap)
{
    size_t n=0; spaces(p); if(*(*p)++!='"') return false;
    while(**p && **p!='"') {
        if((unsigned char)**p<32U || n+1>=cap) return false;
        out[n++]=*(*p)++;
    }
    if(**p!='"') return false;
    ++*p; spaces(p); out[n]=0; return true;
}
static bool copy_field(char *out, size_t cap, const char *s)
{
    size_t n=strlen(s);
    if(!n||n>=cap) return false;
    memcpy(out,s,n+1); return true;
}
static bool safe_field(const char *s, size_t max)
{
    size_t i;
    if(!s||!s[0]) return false;
    for(i=0;s[i];++i)
        if(i>=max || (unsigned char)s[i]<32U || (unsigned char)s[i]>126U ||
           s[i]=='"' || s[i]=='\\') return false;
    return true;
}
static bool registered(const ML307C *m)
{ return m->status.registration==1U || m->status.registration==5U; }
static bool https_configured(const ML307C *m)
{ return m->http_config.host && !strncmp(m->http_config.host,"https://",8); }
static ML307C_Result map_result(ATClient_Result r)
{
    if(r==ATCLIENT_RESULT_OK) return ML307C_RESULT_OK;
    if(r==ATCLIENT_RESULT_TIMEOUT) return ML307C_RESULT_TIMEOUT;
    if(r==ATCLIENT_RESULT_PROTOCOL_ERROR || r==ATCLIENT_RESULT_LINE_TOO_LONG ||
       r==ATCLIENT_RESULT_RX_OVERFLOW) return ML307C_RESULT_PARSE_ERROR;
    return ML307C_RESULT_AT_ERROR;
}
static void lost(ML307C *m)
{
    m->status.data_active=false;
    m->status.tls_ready=false;
    m->status.ipv4[0]=m->status.ipv6[0]=0;
    m->network_lost=true;
    if(!m->sms_channel_uncertain) state(m,ML307C_STATE_NETWORK_WAIT);
    /* A disconnected modem cannot finish an in-flight SMS transaction.
     * Wake ATClient so the main loop reports failure without waiting 120 s. */
    if(m->status.sms_busy && m->pending) ATClient_OnTransportError(m->at);
}
static void urc(void *ctx, const char *s, size_t len)
{
    ML307C *m=ctx;
    const char *p;
    uint32_t a,b,c,d,e;
    bool ok=false;
    (void)len;
    if(!strcmp(s,"+MATREADY")) {
        m->status.matready_seen=true;
        /* During first probe MATREADY is expected; afterwards it invalidates
         * all server and PDP handles. In-flight SMS is aborted by lost(). */
        if(m->startup_notified || m->status.http_busy) { m->reboot=true; lost(m); }
        return;
    }
    if(!strncmp(s,"+CEREG:",7)) {
        p=s+7;
        if(number(&p,&a)) {
            b=a;
            if(*p==',') {
                ++p; spaces(&p);
                /* Query: <n>,<stat>; notification: <stat>,"tac",... */
                if(*p!='"' && !number(&p,&b)) goto malformed;
            }
            if(b<=10 && (!*p || *p==',' || *p=='"')) {
                m->status.registration=(uint8_t)b;
                if(m->stage==ML_N_REG && m->pending) m->value_valid=true;
                if(!registered(m) && m->status.data_active) lost(m);
                return;
            }
        }
    } else if(!strncmp(s,"+MIPCALL:",9)) {
        char ip1[46]={0},ip2[46]={0},v4[16]={0},v6[46]={0};
        p=s+9;
        if(!number(&p,&a)||!comma(&p)||!number(&p,&b)||b>1) goto malformed;
        if(a!=1) return; /* Only CID 1 belongs to this driver. */
        if(b==1) {
            if(!comma(&p)||!quoted(&p,ip1,sizeof(ip1))) goto malformed;
            if(*p && (!comma(&p)||!quoted(&p,ip2,sizeof(ip2)))) goto malformed;
            if(*p) goto malformed;
            const char *ips[2]={ip1,ip2};
            for(unsigned i=0;i<2;++i) {
                if(!ips[i][0]) continue;
                if(strchr(ips[i],':')) { if(!copy_field(v6,sizeof(v6),ips[i])) goto malformed; }
                else if(!strchr(ips[i],'.') || !copy_field(v4,sizeof(v4),ips[i])) goto malformed;
            }
            if(!v4[0]&&!v6[0]) goto malformed;
        } else if(*p) goto malformed;
        memcpy(m->status.ipv4,v4,sizeof(v4)); memcpy(m->status.ipv6,v6,sizeof(v6));
        m->status.data_active=(b==1);
        m->pdp_notice=true;
        if(m->stage==ML_N_PDP && m->pending) m->value_valid=true;
        if(!b && m->startup_notified) lost(m);
        return;
    } else if(!strncmp(s,"+MHTTPURC:",10)) {
        char event[16]; p=s+10;
        if(!quoted(&p,event,sizeof(event))||!comma(&p)||!number(&p,&a)) goto malformed;
        if(!m->status.http_busy || (int)a!=m->http_id) return;
        if(!strcmp(event,"recv")) {
            if(!comma(&p)||!number(&p,&b)||!comma(&p)||!number(&p,&c)||
               !comma(&p)||!number(&p,&d)||*p || b<100 || b>599 ||
               c>CACHE_BYTES || d>CACHE_BYTES || c+d>CACHE_BYTES) goto malformed;
            if(m->stage!=ML_H_REQUEST && m->stage!=ML_H_WAIT) return;
            if(m->response_seen) return;
            m->response_seen=true; m->status.http_status=(uint16_t)b;
            m->remaining[0]=c; m->remaining[1]=d;
            if(m->callbacks.on_http_response)
                m->callbacks.on_http_response(m->callback_context,(uint16_t)b,c,d);
            return;
        }
        if(!strcmp(event,"err")) {
            ok=comma(&p)&&number(&p,&e)&&!*p;
            if(!ok) goto malformed;
            m->status.http_error=e; m->http_result=ML307C_RESULT_HTTP_ERROR;
            return;
        }
        return;
    } else return;
malformed:
    ++m->status.urc_error_count; ++m->status.parse_error_count;
    if(m->pending) m->bad_value=true;
    if(m->status.http_busy) m->http_result=ML307C_RESULT_PARSE_ERROR;
}
static void on_line(void *ctx, const char *s, size_t len)
{
    ML307C *m=ctx; const char *p=s; uint32_t a,b; bool ok=false;
    (void)len;
    switch(m->stage) {
    case ML_T_CLOCK:
        if(strncmp(s,"+CCLK: \"",8)) return;
        p=s+8;
        /* The modem starts with an invalid clock. Only network-sourced
         * dates from 2024 onward are useful for certificate expiry checks. */
        ok=strlen(p)==21 && p[0]>='0'&&p[0]<='6' && p[1]>='0'&&p[1]<='9' &&
            (p[0]>'2' || (p[0]=='2' && p[1]>='4')) &&
            p[2]=='/' && p[5]=='/' && p[8]==',' && p[11]==':' &&
            p[14]==':' && (p[17]=='+'||p[17]=='-') && p[20]=='"';
        for(size_t i=3;ok&&i<20;++i)
            if(i!=5&&i!=8&&i!=11&&i!=14&&i!=17 && (p[i]<'0'||p[i]>'9')) ok=false;
        if(ok) {
            unsigned year=(unsigned)(p[0]-'0')*10U+(unsigned)(p[1]-'0');
            unsigned month=(unsigned)(p[3]-'0')*10U+(unsigned)(p[4]-'0');
            unsigned day=(unsigned)(p[6]-'0')*10U+(unsigned)(p[7]-'0');
            unsigned hour=(unsigned)(p[9]-'0')*10U+(unsigned)(p[10]-'0');
            unsigned minute=(unsigned)(p[12]-'0')*10U+(unsigned)(p[13]-'0');
            unsigned second=(unsigned)(p[15]-'0')*10U+(unsigned)(p[16]-'0');
            unsigned zone=(unsigned)(p[18]-'0')*10U+(unsigned)(p[19]-'0');
            static const uint8_t days[]={0,31,28,31,30,31,30,31,31,30,31,30,31};
            unsigned max_day=month>=1U&&month<=12U?days[month]:0U;
            if(month==2U && year%4U==0U) max_day=29U;
            ok=day>=1U&&day<=max_day&&hour<24U&&minute<60U&&second<60U&&zone<=56U;
        }
        break;
    case ML_T_LIST:
        if(strncmp(s,"+MSSLLIST:",10)) return;
        {
            char name[64];
            p=s+10;
            ok=quoted(&p,name,sizeof(name))&&comma(&p)&&number(&p,&a)&&!*p;
            if(ok && !strcmp(name,m->http_config.ca_name) && a==m->http_config.ca_length)
                m->tls_cert_present=true;
        }
        break;
    case ML_N_MODEL:
        if(!strncmp(s,"Model:",6)) { p=s+6; spaces(&p); }
        else if(strncmp(s,"ML307",5)) return;
        ok=copy_field(m->status.model,sizeof(m->status.model),p); break;
    case ML_N_FW:
        if(!strncmp(s,"+CGMR:",6)) { p=s+6; spaces(&p); }
        ok=copy_field(m->status.firmware,sizeof(m->status.firmware),p); break;
    case ML_N_IMEI:
        if(!strncmp(s,"+CGSN:",6)) { p=s+6; spaces(&p); }
        ok=strlen(p)==15;
        for(size_t i=0;ok&&i<15;++i) if(p[i]<'0'||p[i]>'9') ok=false;
        if(ok) memcpy(m->status.imei,p,16);
        break;
    case ML_S_SEND:
        if(strncmp(s,"+CMGS:",6)) return;
        p=s+6; ok=number(&p,&a)&&!*p&&a<=255U;
        if(ok) m->status.sms_reference=a;
        break;
    case ML_N_SIM:
        if(strncmp(s,"+CPIN:",6)) return;
        p=s+6; spaces(&p); m->status.sim_ready=!strcmp(p,"READY"); ok=*p!=0; break;
    case ML_N_CFUN:
        if(strncmp(s,"+CFUN:",6)) return;
        p=s+6; ok=number(&p,&a)&&!*p&&a<=127;
        if(ok) m->status.cfun=(uint8_t)a;
        break;
    case ML_N_CSQ:
        if(strncmp(s,"+CSQ:",5)) return;
        p=s+5; ok=number(&p,&a)&&comma(&p)&&number(&p,&b)&&!*p&&
            (a<=31||a==99)&&(b<=7||b==99);
        if(ok) {
            m->status.rssi=(uint8_t)a; m->status.ber=(uint8_t)b;
            m->status.rssi_known=a!=99;
            m->status.rssi_dbm=a==99?0:(int16_t)(2*(int)a-113);
        } break;
    case ML_H_CREATE:
        if(strncmp(s,"+MHTTPCREATE:",13)) return;
        p=s+13; ok=number(&p,&a)&&!*p&&a<=3;
        if(ok) m->http_id=(int)a;
        break;
    default: return;
    }
    if(ok) m->value_valid=true;
    else { m->bad_value=true; ++m->status.parse_error_count; }
}
static void on_complete(void *ctx, ATClient_Result r)
{ ML307C *m=ctx; m->pending=false; m->done=true; m->at_result=r; }

/* Probe is called for each header byte by ATClient. Once the fourth comma
 * arrives, raw data_len bytes (including NUL/newlines) belong to this frame. */
static ATClient_FrameProbeResult probe(void *ctx, const uint8_t *h, size_t n, size_t *length)
{
    static const char prefix[]="+MHTTPREAD:";
    const size_t plen=sizeof(prefix)-1;
    char text[96]; const char *p; uint32_t a,b,c,d; unsigned commas=0;
    (void)ctx;
    if(memcmp(h,prefix,n<plen?n:plen)) return ATCLIENT_FRAME_NO_MATCH;
    if(n<plen) return ATCLIENT_FRAME_NEED_MORE;
    for(size_t i=plen;i<n;++i) if(h[i]==',') ++commas;
    if(commas<4) return ATCLIENT_FRAME_NEED_MORE;
    if(n>=sizeof(text)) return ATCLIENT_FRAME_ERROR;
    memcpy(text,h,n); text[n]=0; p=text+plen;
    if(!number(&p,&a)||!comma(&p)||!number(&p,&b)||!comma(&p)||
       !number(&p,&c)||!comma(&p)||!number(&p,&d)||!comma(&p)||*p ||
       a>3 || b>1 || d>ATCLIENT_FRAME_PAYLOAD_MAX) return ATCLIENT_FRAME_ERROR;
    *length=d; return ATCLIENT_FRAME_READY;
}
static void frame_begin(void *ctx, const uint8_t *h, size_t n, size_t length)
{
    ML307C *m=ctx; char text[96]; const char *p; uint32_t a=0,b=0,c=0,d=0;
    memcpy(text,h,n); text[n]=0; p=text+11;
    (void)number(&p,&a); (void)comma(&p); (void)number(&p,&b);
    (void)comma(&p); (void)number(&p,&c); (void)comma(&p); (void)number(&p,&d);
    m->frame_seen=true; m->frame_size=length; m->frame_unread=c; m->frame_received=0;
    m->frame_complete=false;
    m->frame_valid=m->pending && m->stage==ML_H_READ && (int)a==m->http_id &&
        b==m->read_type && length>0 && length<=m->read_size &&
        length<=m->remaining[m->read_type] &&
        c==m->remaining[m->read_type]-length;
    if(!m->frame_valid) m->http_result=ML307C_RESULT_PARSE_ERROR;
}
static void frame_data(void *ctx, const uint8_t *d, size_t n)
{
    ML307C *m=ctx;
    m->frame_received+=n;
    if(!m->frame_valid || m->http_result!=ML307C_RESULT_OK || m->network_lost) return;
    if(m->read_type==1) m->status.http_received_bytes+=(uint32_t)n;
    if(m->callbacks.on_http_data)
        m->callbacks.on_http_data(m->callback_context,(ML307C_HttpDataType)m->read_type,d,n);
}
static void frame_end(void *ctx, ATClient_Result r)
{
    ML307C *m=ctx;
    m->frame_complete=(r==ATCLIENT_RESULT_OK && m->frame_received==m->frame_size);
}
bool ML307C_Init(ML307C *m, ATClient *at, const ML307C_Callbacks *cb, void *ctx)
{
    ATClient_LengthFrameRegistration f={"+MHTTPREAD:",probe,frame_begin,frame_data,frame_end,m};
    if(!m||!at) return false;
    memset(m,0,sizeof(*m)); m->at=at; if(cb) m->callbacks=*cb;
    m->callback_context=ctx; m->http_id=-1;
    return ATClient_RegisterUrc(at,"+MATREADY",urc,m)==ATCLIENT_RESULT_OK &&
        ATClient_RegisterUrc(at,"+CEREG:",urc,m)==ATCLIENT_RESULT_OK &&
        ATClient_RegisterUrc(at,"+MIPCALL:",urc,m)==ATCLIENT_RESULT_OK &&
        ATClient_RegisterUrc(at,"+MHTTPURC:",urc,m)==ATCLIENT_RESULT_OK &&
        ATClient_RegisterLengthFrame(at,&f)==ATCLIENT_RESULT_OK;
}
ML307C_Result ML307C_Start(ML307C *m)
{
    if(!m||!m->at) return ML307C_RESULT_INVALID_ARGUMENT;
    if(m->started || m->pending) return ML307C_RESULT_BUSY;
    m->started=true; m->stage=ML_N_AT; state(m,ML307C_STATE_PROBING);
    return ML307C_RESULT_OK;
}
ML307C_State ML307C_GetState(const ML307C *m) { return m?m->state:ML307C_STATE_OFFLINE; }
const ML307C_Status *ML307C_GetStatus(const ML307C *m) { return m?&m->status:NULL; }
void ML307C_RequestIdleSleep(ML307C *m, bool enabled)
{
    if(m) m->sleep_requested=enabled;
}
bool ML307C_SleepReady(const ML307C *m)
{
    return m && m->sleep_requested && m->sleep_configured && !m->sleeping &&
        !m->pending && m->stage==ML_N_IDLE && !m->status.http_busy &&
        !m->status.sms_busy && !m->network_lost && !m->reboot &&
        m->state==ML307C_STATE_NETWORK_READY && m->status.data_active &&
        (!https_configured(m) || m->status.tls_ready) && ATClient_IsIdle(m->at);
}
bool ML307C_EnterSleep(ML307C *m)
{
    if(!ML307C_SleepReady(m)) return false;
    m->sleeping=true;
    return true;
}
void ML307C_Wake(ML307C *m)
{
    if(!m || !m->sleeping) return;
    m->sleeping=false; m->sleep_requested=false;
    if(m->reboot) m->sleep_configured=false;
    m->retry_wait=false; m->network_lost=false; m->reboot=false;
    m->status.responsive=false; m->status.data_active=false;
    m->status.tls_ready=false; m->status.registration=0;
    m->status.ipv4[0]=m->status.ipv6[0]=0;
    m->stage=ML_N_AT;
    state(m,ML307C_STATE_PROBING);
}
bool ML307C_HttpConfigure(ML307C *m, const ML307C_HttpConfig *cfg)
{
    size_t scheme;
    if(!m||!cfg||m->status.http_busy||!safe_field(cfg->host,192)||
       cfg->read_chunk_size>1024) return false;
    if(!strncmp(cfg->host,"https://",8)) {
        scheme=8;
        if(!cfg->ca_pem || !cfg->ca_length || cfg->ca_length>8192 ||
           !safe_field(cfg->ca_name,63)) return false;
    } else if(!strncmp(cfg->host,"http://",7)) scheme=7;
    else return false;
    if(!cfg->host[scheme]) return false;
    /* Host is an authority only; path belongs to each request. */
    for(const char *p=cfg->host+scheme;*p;++p)
        if(*p=='/'||*p=='?'||*p=='#'||*p==' '||*p=='@') return false;
    m->http_config=*cfg;
    m->status.tls_ready=false;
    if(!m->http_config.read_chunk_size) m->http_config.read_chunk_size=256;
    return true;
}
static ML307C_Result http_start(ML307C *m, const char *path, unsigned method)
{
    if(!m||!safe_field(path,256)||path[0]!='/') return ML307C_RESULT_INVALID_ARGUMENT;
    if(https_configured(m) && !m->status.tls_ready) return ML307C_RESULT_NOT_READY;
    if(m->sleeping) return ML307C_RESULT_NOT_READY;
    if(m->state==ML307C_STATE_PROBING) return ML307C_RESULT_NOT_READY;
    if(m->status.http_busy||m->pending||m->stage!=ML_N_IDLE) return ML307C_RESULT_BUSY;
    /* A failed delete leaves ownership ambiguous. Require a modem restart
     * before allocating another instance instead of leaking all four IDs. */
    if(m->status.cleanup_result!=ML307C_RESULT_OK) return ML307C_RESULT_NOT_READY;
    if(!m->http_config.host || m->state!=ML307C_STATE_NETWORK_READY ||
       !m->status.data_active || !registered(m)) return ML307C_RESULT_NOT_READY;
    m->path=path; m->method=method; m->http_id=-1; m->response_seen=false;
    m->http_result=ML307C_RESULT_OK; m->status.http_busy=true;
    m->status.http_status=0; m->status.http_error=0; m->status.http_received_bytes=0;
    m->status.cleanup_result=ML307C_RESULT_OK;
    ++m->status.http_request_count; m->stage=ML_H_CREATE;
    return ML307C_RESULT_OK;
}
ML307C_Result ML307C_HttpGet(ML307C *m, const char *path)
{ return http_start(m,path,1); }
ML307C_Result ML307C_HttpPost(ML307C *m, const char *path, const char *type,
                             const uint8_t *body, size_t length)
{
    ML307C_Result r;
    if(!safe_field(type,96)||!body||!length||length>4096) return ML307C_RESULT_INVALID_ARGUMENT;
    r=http_start(m,path,2);
    if(r!=ML307C_RESULT_OK) return r;
    (void)snprintf(m->header,sizeof(m->header),"Content-Type: %s\r\n",type);
    m->body=body; m->body_length=length; return r;
}
ML307C_Result ML307C_SendSms(ML307C *m, const char *number, const char *text)
{
    size_t n=0, length=0;
    if(!m||!number||!text) return ML307C_RESULT_INVALID_ARGUMENT;
    if(number[0]=='+') n=1;
    if(number[n]<'0'||number[n]>'9') return ML307C_RESULT_INVALID_ARGUMENT;
    for(;number[n];++n)
        if(n>=20U||number[n]<'0'||number[n]>'9') return ML307C_RESULT_INVALID_ARGUMENT;
    for(;text[length];++length)
        if(length>=160U||(unsigned char)text[length]<32U||
           (unsigned char)text[length]>126U) return ML307C_RESULT_INVALID_ARGUMENT;
    if(!length) return ML307C_RESULT_INVALID_ARGUMENT;
    if(m->sleeping) return ML307C_RESULT_NOT_READY;
    if(m->status.sms_busy||m->status.http_busy||m->pending||m->stage!=ML_N_IDLE)
        return ML307C_RESULT_BUSY;
    if(m->state!=ML307C_STATE_NETWORK_READY || !m->status.sim_ready ||
       !registered(m)) return ML307C_RESULT_NOT_READY;
    memcpy(m->sms_number,number,n+1);
    memcpy(m->sms_payload,text,length);
    m->sms_payload[length]=0x1AU; m->sms_payload_length=length+1U;
    m->status.sms_reference=0; m->status.sms_busy=true;
    m->stage=ML_S_MODE;
    return ML307C_RESULT_OK;
}
static bool build(void *ctx, char *dst, size_t cap, size_t *length)
{
    ML307C *m=ctx; int n=-1;
    /* Bare AT+CGSN returns ML307C's alphanumeric serial, not its IMEI. */
    static const char *const commands[]={
        "", "AT","ATE0","AT+CMEE=2","ATI","AT+CGMR","AT+CGSN=1",
        "AT+CPIN?","AT+CFUN?","AT+CSQ","AT+CEREG?","AT+MIPCALL?","AT+MIPCALL=1,1"
    };
    if(m->stage>=ML_N_AT && m->stage<=ML_N_ACTIVATE)
        n=snprintf(dst,cap,"%s",commands[m->stage]);
    else switch(m->stage) {
    case ML_T_CLOCK: n=snprintf(dst,cap,"AT+CCLK?"); break;
    case ML_T_LIST: n=snprintf(dst,cap,"AT+MSSLLIST=1"); break;
    case ML_T_WRITE: n=snprintf(dst,cap,"AT+MSSLCERTWR=\"%s\",0,%lu",m->http_config.ca_name,(unsigned long)m->http_config.ca_length); break;
    case ML_T_CERT: n=snprintf(dst,cap,"AT+MSSLCFG=\"cert\",1,\"%s\"",m->http_config.ca_name); break;
    case ML_T_AUTH: n=snprintf(dst,cap,"AT+MSSLCFG=\"auth\",1,1"); break;
    case ML_T_STAMP: n=snprintf(dst,cap,"AT+MSSLCFG=\"ignorestamp\",1,0"); break;
    case ML_T_VERIFY: n=snprintf(dst,cap,"AT+MSSLCFG=\"ignoreverify\",1,0"); break;
    case ML_T_SNI: n=snprintf(dst,cap,"AT+MSSLCFG=\"sni\",1,1"); break;
    case ML_H_CREATE: n=snprintf(dst,cap,"AT+MHTTPCREATE=\"%s\"",m->http_config.host); break;
    case ML_H_SSL: n=snprintf(dst,cap,"AT+MHTTPCFG=\"ssl\",%d,1,1",m->http_id); break;
    case ML_H_CACHE: n=snprintf(dst,cap,"AT+MHTTPCFG=\"cached\",%d,1,4096",m->http_id); break;
    case ML_H_ENCODING: n=snprintf(dst,cap,"AT+MHTTPCFG=\"encoding\",%d,0,0",m->http_id); break;
    case ML_H_TIMEOUT: n=snprintf(dst,cap,"AT+MHTTPCFG=\"timeout\",%d,30,30,10",m->http_id); break;
    case ML_H_HEADER: n=snprintf(dst,cap,"AT+MHTTPHEADER=%d,0,%lu",m->http_id,(unsigned long)strlen(m->header)); break;
    case ML_H_BODY: n=snprintf(dst,cap,"AT+MHTTPCONTENT=%d,0,%lu",m->http_id,(unsigned long)m->body_length); break;
    case ML_H_REQUEST: n=snprintf(dst,cap,"AT+MHTTPREQUEST=%d,%u,0,\"%s\"",m->http_id,m->method,m->path); break;
    case ML_H_READ: n=snprintf(dst,cap,"AT+MHTTPREAD=%d,%u,%lu",m->http_id,m->read_type,(unsigned long)m->read_size); break;
    case ML_H_DELETE: n=snprintf(dst,cap,"AT+MHTTPDEL=%d",m->http_id); break;
    case ML_S_MODE: n=snprintf(dst,cap,"AT+CMGF=1"); break;
    case ML_S_SEND: n=snprintf(dst,cap,"AT+CMGS=\"%s\"",m->sms_number); break;
    case ML_P_SLEEP: n=snprintf(dst,cap,"AT+MLPMCFG=\"sleepmode\",2,0"); break;
    default: break;
    }
    if(n<=0||(size_t)n>=cap) return false;
    *length=(size_t)n; return true;
}
static void retry(ML307C *m, ML307C_Result r, ML307C_Stage next)
{
    m->status.last_result=r; ++m->status.retry_count;
    m->retry_wait=true; m->due_ms=ATClient_NowMs(m->at)+5000U;
    m->stage=next; state(m,ML307C_STATE_RETRY_WAIT);
}
static void ready(ML307C *m)
{
    m->network_lost=false; m->stage=ML_N_IDLE; m->health_ms=ATClient_NowMs(m->at);
    m->status.last_check_ms=m->health_ms; m->status.last_result=ML307C_RESULT_OK;
    state(m,ML307C_STATE_NETWORK_READY);
    if(!m->startup_notified) {
        m->startup_notified=true;
        if(m->callbacks.on_startup_complete) m->callbacks.on_startup_complete(m->callback_context,ML307C_RESULT_OK);
    }
}
static void http_finish(ML307C *m)
{
    ML307C_Result r=m->http_result;
    m->status.last_result=r; m->status.http_busy=false; m->http_id=-1;
    m->stage=ML_N_IDLE;
    /* Prevent immediate status traffic after a failed request; never replay
     * a POST whose server-side outcome is unknown. */
    m->health_ms=ATClient_NowMs(m->at);
    if(m->callbacks.on_http_complete) m->callbacks.on_http_complete(m->callback_context,r);
}
static void sms_finish(ML307C *m, ML307C_Result r)
{
    /* A timed-out CMGS may still be in the module's '>' input mode. Sending
     * AT+CSQ now could append it to the SMS. Wait for a confirmed reset URC. */
    if(m->stage==ML_S_SEND &&
       (r==ML307C_RESULT_TIMEOUT || r==ML307C_RESULT_PARSE_ERROR ||
        (r==ML307C_RESULT_NETWORK_LOST && !m->reboot) ||
        (m->at_result==ATCLIENT_RESULT_TRANSPORT_ERROR && !m->reboot))) {
        m->sms_channel_uncertain=true;
        m->status.responsive=false;
        state(m,ML307C_STATE_ERROR);
    }
    m->status.last_result=r; m->status.sms_busy=false; m->stage=ML_N_IDLE;
    if(r!=ML307C_RESULT_OK) m->status.sms_reference=0;
    m->health_ms=ATClient_NowMs(m->at);
    if(m->callbacks.on_sms_complete)
        m->callbacks.on_sms_complete(m->callback_context,r,m->status.sms_reference);
}
static void choose_read(ML307C *m)
{
    if(m->remaining[0]) m->read_type=0;
    else if(m->remaining[1]) m->read_type=1;
    else { m->stage=ML_H_DELETE; return; }
    m->read_size=m->remaining[m->read_type];
    if(m->read_size>m->http_config.read_chunk_size) m->read_size=m->http_config.read_chunk_size;
    m->stage=ML_H_READ;
}
static void handle_done(ML307C *m)
{
    ML307C_Result r=map_result(m->at_result);
    bool needs_value=(m->stage>=ML_N_MODEL && m->stage<=ML_N_PDP)||
                     m->stage==ML_T_CLOCK||m->stage==ML_H_CREATE||m->stage==ML_S_SEND;
    if(r==ML307C_RESULT_OK && (m->bad_value || (needs_value&&!m->value_valid)))
        r=ML307C_RESULT_PARSE_ERROR;
    if(m->stage==ML_P_SLEEP) {
        m->status.last_result=r;
        if(r==ML307C_RESULT_OK) m->sleep_configured=true;
        else m->sleep_due_ms=ATClient_NowMs(m->at)+5000U;
        m->stage=ML_N_IDLE;
        m->health_ms=ATClient_NowMs(m->at);
        return;
    }
    if(m->status.sms_busy) {
        if(m->network_lost) r=ML307C_RESULT_NETWORK_LOST;
        if(r!=ML307C_RESULT_OK) sms_finish(m,r);
        else if(m->stage==ML_S_MODE) m->stage=ML_S_SEND;
        else sms_finish(m,ML307C_RESULT_OK);
        return;
    }
    if(m->stage>=ML_T_CLOCK && m->stage<=ML_T_SNI) {
        if(m->network_lost) r=ML307C_RESULT_NETWORK_LOST;
        if(r!=ML307C_RESULT_OK) {
            m->status.tls_ready=false;
            m->status.last_result=r;
            m->tls_due_ms=ATClient_NowMs(m->at)+5000U;
            if(m->stage==ML_T_WRITE && (r==ML307C_RESULT_TIMEOUT ||
                m->at_result==ATCLIENT_RESULT_TRANSPORT_ERROR)) {
                m->tls_channel_uncertain=true;
                state(m,ML307C_STATE_ERROR);
            }
            m->stage=ML_N_IDLE;
        } else if(m->stage==ML_T_LIST) {
            m->stage=m->tls_cert_present?ML_T_CERT:ML_T_WRITE;
        } else if(m->stage==ML_T_SNI) {
            m->status.tls_ready=true;
            m->status.last_result=ML307C_RESULT_OK;
            m->stage=ML_N_IDLE;
            m->health_ms=ATClient_NowMs(m->at);
        } else m->stage=(ML307C_Stage)(m->stage+1);
        return;
    }
    if(m->status.http_busy) {
        if(m->network_lost) m->http_result=ML307C_RESULT_NETWORK_LOST;
        if(m->stage==ML_H_DELETE) {
            m->status.cleanup_result=r;
            if(m->http_result==ML307C_RESULT_OK) m->http_result=r;
            http_finish(m); return;
        }
        if(m->http_result==ML307C_RESULT_OK && r!=ML307C_RESULT_OK) m->http_result=r;
        if(m->http_result!=ML307C_RESULT_OK) return; /* process schedules cleanup */
        switch(m->stage) {
        case ML_H_CREATE: m->stage=https_configured(m)?ML_H_SSL:ML_H_CACHE; break;
        case ML_H_SSL: m->stage=ML_H_CACHE; break;
        case ML_H_CACHE: m->stage=ML_H_ENCODING; break;
        case ML_H_ENCODING: m->stage=ML_H_TIMEOUT; break;
        case ML_H_TIMEOUT: m->stage=m->method==2?ML_H_HEADER:ML_H_REQUEST; break;
        case ML_H_HEADER: m->stage=ML_H_BODY; break;
        case ML_H_BODY: m->stage=ML_H_REQUEST; break;
        case ML_H_REQUEST: m->stage=ML_H_WAIT; m->wait_ms=ATClient_NowMs(m->at); break;
        case ML_H_READ:
            if(!m->frame_seen||!m->frame_valid||!m->frame_complete)
                m->http_result=ML307C_RESULT_PARSE_ERROR;
            else { m->remaining[m->read_type]=m->frame_unread; choose_read(m); }
            break;
        default: break;
        }
        return;
    }
    if(r!=ML307C_RESULT_OK) {
        if(r==ML307C_RESULT_AT_ERROR || r==ML307C_RESULT_TIMEOUT) m->status.responsive=false;
        if(m->startup_notified) lost(m);
        retry(m,r,m->startup_notified?ML_N_AT:m->stage); return;
    }
    m->status.responsive=true;
    switch(m->stage) {
    case ML_N_SIM:
        if(!m->status.sim_ready) { state(m,ML307C_STATE_SIM_WAIT); retry(m,ML307C_RESULT_NOT_READY,ML_N_SIM); }
        else m->stage=ML_N_CFUN;
        break;
    case ML_N_CFUN:
        if(m->status.cfun!=1) retry(m,ML307C_RESULT_NOT_READY,ML_N_SIM);
        else m->stage=ML_N_CSQ;
        break;
    case ML_N_REG:
        if(!registered(m)) { state(m,ML307C_STATE_NETWORK_WAIT); retry(m,ML307C_RESULT_NOT_READY,ML_N_SIM); }
        else m->stage=ML_N_PDP;
        break;
    case ML_N_PDP:
        if(m->status.data_active) ready(m);
        else { m->stage=ML_N_ACTIVATE; m->pdp_notice=false; state(m,ML307C_STATE_DATA_ACTIVATING); }
        break;
    case ML_N_ACTIVATE: m->stage=ML_N_WAIT; m->wait_ms=ATClient_NowMs(m->at); break;
    default: m->stage=(ML307C_Stage)(m->stage+1); break;
    }
}
void ML307C_Process(ML307C *m)
{
    ATClient_Request req={0}; uint32_t now_ms;
    if(!m||!m->started || m->sleeping) return;
    now_ms=ATClient_NowMs(m->at);
    /* No response polling loops: pending transactions return immediately.
     * ATClient invokes completion/URC callbacks on subsequent main iterations. */
    if(m->pending) return;
    if(m->reboot) {
        m->done=false;
        if(m->status.http_busy) { m->http_result=ML307C_RESULT_NETWORK_LOST; http_finish(m); }
        if(m->status.sms_busy) sms_finish(m,ML307C_RESULT_NETWORK_LOST);
        m->reboot=false; m->sms_channel_uncertain=false; m->tls_channel_uncertain=false;
        m->sleep_configured=false;
        m->status.tls_ready=false;
        m->status.cleanup_result=ML307C_RESULT_OK;
        m->startup_notified=false; retry(m,ML307C_RESULT_NETWORK_LOST,ML_N_AT);
    }
    if(m->done) { m->done=false; handle_done(m); }
    if(m->sms_channel_uncertain || m->tls_channel_uncertain) return;
    if(m->retry_wait) {
        if((int32_t)(now_ms-m->due_ms)<0) return;
        m->retry_wait=false;
    }
    if(m->status.http_busy) {
        if(m->network_lost) m->http_result=ML307C_RESULT_NETWORK_LOST;
        if(m->http_result!=ML307C_RESULT_OK && m->stage!=ML_H_DELETE) {
            if(m->http_id<0) { http_finish(m); return; }
            m->stage=ML_H_DELETE;
        }
        if(m->stage==ML_H_WAIT) {
            if(m->response_seen) choose_read(m);
            else if((uint32_t)(now_ms-m->wait_ms)>=HTTP_MS) {
                m->http_result=ML307C_RESULT_TIMEOUT; m->stage=ML_H_DELETE;
            } else return;
        }
    } else if(m->status.sms_busy) {
        if(m->network_lost) { sms_finish(m,ML307C_RESULT_NETWORK_LOST); }
    } else if(m->stage==ML_N_IDLE) {
        if(m->network_lost) { m->network_lost=false; retry(m,ML307C_RESULT_NETWORK_LOST,ML_N_SIM); return; }
        if(https_configured(m) && !m->status.tls_ready &&
           m->state==ML307C_STATE_NETWORK_READY && m->status.data_active &&
           (int32_t)(now_ms-m->tls_due_ms)>=0) {
            m->stage=ML_T_CLOCK;
        } else {
            if(m->sleep_requested && m->state==ML307C_STATE_NETWORK_READY &&
               m->status.data_active &&
               (!https_configured(m) || m->status.tls_ready)) {
                if(m->sleep_configured) return;
                if((int32_t)(now_ms-m->sleep_due_ms)>=0) m->stage=ML_P_SLEEP;
                else return;
            }
            if(m->stage!=ML_P_SLEEP) {
                if((uint32_t)(now_ms-m->health_ms)<HEALTH_MS) return;
                m->stage=ML_N_CSQ;
            }
        }
    } else if(m->stage==ML_N_WAIT) {
        if(m->pdp_notice&&m->status.data_active&&registered(m)) { ready(m); return; }
        if((uint32_t)(now_ms-m->wait_ms)>=HTTP_MS)
            retry(m,ML307C_RESULT_TIMEOUT,ML_N_SIM);
        return;
    }
    if(m->stage==ML_N_IDLE || !ATClient_IsIdle(m->at)) return;
    req.build_command=build; req.command_context=m; req.timeout_ms=COMMAND_MS;
    req.on_line=on_line; req.on_complete=on_complete; req.callback_context=m;
    if(m->stage==ML_H_HEADER || m->stage==ML_H_BODY || m->stage==ML_T_WRITE) {
        /* Longer than the explicitly configured 10 s modem input timeout:
         * don't inject a cleanup AT command while still in data-input mode. */
        req.timeout_ms=15000U;
        req.prompt_byte='>';
        req.payload=m->stage==ML_H_HEADER?(const uint8_t *)m->header:
                    m->stage==ML_H_BODY?m->body:m->http_config.ca_pem;
        req.payload_length=m->stage==ML_H_HEADER?strlen(m->header):
                           m->stage==ML_H_BODY?m->body_length:m->http_config.ca_length;
    } else if(m->stage==ML_S_SEND) {
        /* ATClient waits for '>' and DMA-sends the exact text + Ctrl-Z bytes.
         * A long network wait is a deadline, never a blocking delay. */
        req.timeout_ms=120000U;
        req.prompt_byte='>';
        req.payload=m->sms_payload;
        req.payload_length=m->sms_payload_length;
    }
    m->value_valid=m->bad_value=m->frame_seen=m->frame_complete=false;
    if(ATClient_Submit(m->at,&req)==ATCLIENT_RESULT_OK) m->pending=true;
}
