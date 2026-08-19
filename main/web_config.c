#include "web_config.h"

#include "analog_input.h"
#include "board_config.h"
#include "digital_input.h"
#include "relay_driver.h"
#include "rv3028.h"

#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WEB_NVS_NAMESPACE "webcfg"
#define WEB_BODY_MAX       256
#define WEB_SWITCH_HOLD_MS  3000
#define WEB_SWITCH_DEBOUNCE_MS 50
#define WEB_SCHEDULE_CHECK_MS 1000
#define WEB_SCHEDULE_NVS_KEY "schedule"

typedef struct {
    uint8_t enabled;
    uint8_t on_hour;
    uint8_t on_minute;
    uint8_t off_hour;
    uint8_t off_minute;
} relay_schedule_t;

static const char *TAG = "WEB";
static bool s_sta_has_credentials;
static bool s_sta_connected;
static bool s_switch_initialized;
static bool s_switch_low;
static bool s_switch_raw_low;
static bool s_switch_action_done;
static bool s_web_started;
static TickType_t s_switch_low_since;
static TickType_t s_switch_raw_changed_at;
static TickType_t s_last_schedule_check;
static relay_schedule_t s_schedules[BOARD_RELAY_COUNT];
static SemaphoreHandle_t s_schedule_lock;
static esp_ip4_addr_t s_sta_ip;
static esp_netif_t *s_ap_netif;
static esp_netif_t *s_sta_netif;
static httpd_handle_t s_server;

static void copy_wifi_field(uint8_t *destination, size_t destination_size,
                            const char *source)
{
    size_t length = strnlen(source, destination_size);
    memcpy(destination, source, length);
}

/* Web page source is UTF-8.  Keep commanded output and NQ feedback separate:
 * neither one is proof that the relay contacts have mechanically closed. */
static const char s_index_html[] =
    "<!doctype html><html lang='zh-CN'><head><meta charset='utf-8'>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>继电器Web配置</title><style>"
    "*{box-sizing:border-box}body{margin:0;background:#f3f6fb;color:#18243a;"
    "font:15px Arial,\"Microsoft YaHei\",sans-serif}"
    "header{background:#28579c;color:#fff;padding:16px 20px;font-size:22px}"
    "main{max-width:960px;margin:auto;padding:16px}.card{background:#fff;"
    "border-radius:10px;padding:18px;margin-bottom:14px;box-shadow:0 2px 10px "
    "#23395d18}h2{font-size:18px;margin:0 0 14px}.grid{display:grid;"
    "grid-template-columns:repeat(auto-fit,minmax(170px,1fr));gap:10px}"
    ".item{border:1px solid #dce5f2;border-radius:8px;padding:12px}"
    ".dot{display:inline-block;width:11px;height:11px;border-radius:50%;"
    "background:#aeb8c8;margin-right:7px}.on{background:#32b768}"
    "button{border:0;border-radius:6px;padding:9px 15px;background:#3478d4;"
    "color:#fff;cursor:pointer;margin:5px}.offbtn{background:#8c98a9}"
    "input{width:100%;padding:9px;margin:5px 0 10px;border:1px solid #cbd6e5;"
    "border-radius:6px}input[type=checkbox]{width:auto}.muted{color:#68758a;"
    "font-size:13px}"
    "#msg{position:fixed;right:15px;bottom:15px;background:#26364d;color:#fff;"
    "padding:10px 14px;border-radius:6px;display:none}</style></head><body>"
    "<header>LoRa继电器智控系统 · Web配置</header><main>"
    "<section class='card'><h2>网络配置</h2><div id='wifi' class='muted'>"
    "正在读取…</div><form id='wf'><label>Wi-Fi名称（SSID）</label>"
    "<input name='ssid' maxlength='32' required><label>Wi-Fi密码</label>"
    "<input name='password' type='password' maxlength='64'>"
    "<button type='submit'>保存并连接</button></form></section>"
    "<section class='card'><h2>继电器控制</h2>"
    "<div class='muted'>NQ不是24V电源检测：关闭命令时NQ未动作属于正常；只有开启命令后才应收到NQ驱动反馈。本机硬件无法检测触点是否实际吸合。</div>"
    "<div id='relays' class='grid'></div></section>"
    "<section class='card'><h2>继电器每日定时"
    "</h2><div class='muted'>启用后按RV-3028时间自动控制，支持跨午夜。</div>"
    "<div id='schedules' class='grid'></div></section>"
    "<section class='card'><h2>输入检测 "
    "NQ1–NQ4</h2><div id='inputs' class='grid'></div></section>"
    "<section class='card'><h2>模拟量检测</h2><div id='analog' "
    "class='grid'></div></section><section class='card'><h2>设备校时</h2>"
    "<div id='rtc' class='muted'>--</div><button onclick='syncTime()'>"
    "同步浏览器时间</button></section></main><div id='msg'></div><script>"
    "const $=s=>document.querySelector(s);let schedulesRendered=false,"
    "statusLoading=false;async function request(u,o){let c=new AbortController,"
    "t=setTimeout(()=>c.abort(),5000);try{return await fetch(u,Object.assign({},"
    "o,{signal:c.signal}))}finally{clearTimeout(t)}}"
    "function toast(t){let e=$('#msg');"
    "e.textContent=t;e.style.display='block';setTimeout(()=>e.style.display="
    "'none',2200)}async function post(u,o){let r=await request(u,{method:'POST',"
    "headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new "
    "URLSearchParams(o)}),j=await r.json();if(!r.ok)throw Error(j.error||"
    "'操作失败');return j}async function relay(ch,on){try{await post("
    "'/api/relay',{channel:ch,on:on?1:0});load()}catch(e){toast(e.message)}}"
    "async function saveSchedule(ch){let on=$(`#son${ch}`).value.split(':'),"
    "off=$(`#soff${ch}`).value.split(':');try{await post('/api/schedule',{"
    "channel:ch,enabled:$(`#sen${ch}`).checked?1:0,on_hour:on[0],"
    "on_minute:on[1],off_hour:off[0],off_minute:off[1]});toast('定时已保存');"
    "schedulesRendered=false;load()}catch(e){toast(e.message)}}"
    "function card(name,on,buttons=''){return `<div class='item'><div><span "
    "class='dot ${on?'on':''}'></span>${name}</div>${buttons}</div>`}"
    "function feedbackText(cmd,fb){if(cmd&&fb)return '已动作';"
    "if(cmd&&!fb)return '未收到反馈（请检查24V及驱动）';"
    "if(!cmd&&fb)return '关闭命令下仍有效（异常）';return '未动作（正常待机）'}"
    "async function load(){if(statusLoading)return;statusLoading=true;try{let r="
    "await request('/api/status',{cache:'no-store'});if(!r.ok)throw Error();let s="
    "await r.json();$('#wifi').textContent=s.wifi.connected?`已连接 "
    "${s.wifi.ssid}，IP：${s.wifi.ip}`:'未连接路由器；热点地址：192.168.4.1';"
    "$('#relays').innerHTML=[0,1,2,3].map(i=>card(`继电器 ${i+1}<br><span "
    "class='muted'>控制命令：${s.relay_commands&(1<<i)?'开启':'关闭'}<br>NQ驱动反馈："
    "${feedbackText(s.relay_commands&(1<<i),s.relay_feedback&(1<<i))}</span>`,"
    "s.relay_feedback&(1<<i),`<br><button onclick='relay(${i},1)'>开启命令</button><button "
    "class='offbtn' onclick='relay(${i},0)'>关闭命令</button>`)).join('');"
    "if(!schedulesRendered){$('#schedules').innerHTML=s.schedules.map((v,i)=>"
    "`<div class='item'>"
    "<b>继电器 ${i+1}</b><br><label><input id='sen${i}' type='checkbox' "
    "${v[0]?'checked':''}> 启用</label><br>开启命令时间<input id='son${i}' "
    "type='time' value='${String(v[1]).padStart(2,'0')}:${String(v[2])."
    "padStart(2,'0')}'>关闭命令时间<input id='soff${i}' type='time' value='"
    "${String(v[3]).padStart(2,'0')}:${String(v[4]).padStart(2,'0')}'>"
    "<button onclick='saveSchedule(${i})'>保存定时</button></div>`).join('');"
    "schedulesRendered=true}"
    "$('#inputs').innerHTML=[0,1,2,3].map(i=>card(`NQ${i+1}：${s.inputs&"
    "(1<<i)?'有效':'无信号'}`,s.inputs&(1<<i))).join('');$('#analog')."
    "innerHTML=s.voltage.map((v,i)=>`<div class='item'>AN${i+1}<br>"
    "0–10V：<b>${(v/100).toFixed(2)} V</b><br>"
    "4–20mA：<b>${(s.current[i]/100).toFixed(2)} mA</b></div>`).join('');"
    "$('#rtc').textContent=s.rtc.valid?`${s.rtc.year}-${String(s.rtc.month)."
    "padStart(2,'0')}-${String(s.rtc.day).padStart(2,'0')} ${String(s.rtc."
    "hour).padStart(2,'0')}:${String(s.rtc.minute).padStart(2,'0')}:${String("
    "s.rtc.second).padStart(2,'0')}`:'RTC时间无效'}catch(e){toast('状态读取失败')}"
    "finally{statusLoading=false}}"
    "$('#wf').onsubmit=async e=>{e.preventDefault();let f=new FormData(e.target);"
    "try{await post('/api/wifi',{ssid:f.get('ssid'),password:f.get('password')"
    "});toast('已保存，正在连接')}catch(x){toast(x.message)}};async function "
    "syncTime(){let d=new Date;try{await post('/api/time',{year:d.getFullYear()"
    ",month:d.getMonth()+1,day:d.getDate(),hour:d.getHours(),minute:d."
    "getMinutes(),second:d.getSeconds()});toast('校时成功');load()}catch(e){"
    "toast(e.message)}}async function poll(){await load();setTimeout(poll,1000)}poll();"
    "</script></body></html>";

static int hex_value(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    c = (char)tolower((unsigned char)c);
    return c >= 'a' && c <= 'f' ? c - 'a' + 10 : -1;
}

static bool form_get(const char *body, const char *key,
                     char *output, size_t output_size)
{
    size_t key_len = strlen(key);
    for (const char *field = body; *field;) {
        const char *end = strchr(field, '&');
        if (!end) end = field + strlen(field);
        const char *equals = memchr(field, '=', (size_t)(end - field));
        if (equals && (size_t)(equals - field) == key_len &&
            memcmp(field, key, key_len) == 0) {
            size_t written = 0;
            for (const char *p = equals + 1; p < end; ++p) {
                char decoded = *p;
                if (decoded == '+') {
                    decoded = ' ';
                } else if (decoded == '%' && p + 2 < end) {
                    int high = hex_value(p[1]);
                    int low = hex_value(p[2]);
                    if (high < 0 || low < 0) return false;
                    decoded = (char)((high << 4) | low);
                    p += 2;
                }
                if (written + 1 >= output_size) return false;
                output[written++] = decoded;
            }
            output[written] = '\0';
            return true;
        }
        field = *end ? end + 1 : end;
    }
    return false;
}

static bool form_get_int(const char *body, const char *key, int *value)
{
    char text[12];
    if (!form_get(body, key, text, sizeof(text))) return false;
    char *end;
    long parsed = strtol(text, &end, 10);
    if (!text[0] || *end) return false;
    *value = (int)parsed;
    return true;
}

static esp_err_t receive_body(httpd_req_t *req, char *body, size_t size)
{
    if (req->content_len <= 0 || req->content_len >= size) {
        return ESP_ERR_INVALID_SIZE;
    }
    int received = 0;
    while (received < req->content_len) {
        int result = httpd_req_recv(req, body + received,
                                    req->content_len - received);
        if (result <= 0) return ESP_FAIL;
        received += result;
    }
    body[received] = '\0';
    return ESP_OK;
}

static esp_err_t send_json(httpd_req_t *req, const char *json)
{
    httpd_resp_set_type(req, "application/json; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, json);
}

static esp_err_t send_error(httpd_req_t *req, const char *status,
                            const char *message)
{
    char json[128];
    snprintf(json, sizeof(json), "{\"error\":\"%s\"}", message);
    httpd_resp_set_status(req, status);
    return send_json(req, json);
}

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, s_index_html, HTTPD_RESP_USE_STRLEN);
}

static bool schedule_is_valid(const relay_schedule_t *schedule)
{
    return schedule->enabled <= 1 &&
           schedule->on_hour < 24 && schedule->off_hour < 24 &&
           schedule->on_minute < 60 && schedule->off_minute < 60;
}

static esp_err_t save_schedules(const relay_schedule_t *schedules)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WEB_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_blob(handle, WEB_SCHEDULE_NVS_KEY, schedules,
                       sizeof(s_schedules));
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

static void load_schedules(void)
{
    relay_schedule_t stored[BOARD_RELAY_COUNT] = {0};
    nvs_handle_t handle;
    if (nvs_open(WEB_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        memset(s_schedules, 0, sizeof(s_schedules));
        return;
    }
    size_t size = sizeof(stored);
    esp_err_t err = nvs_get_blob(handle, WEB_SCHEDULE_NVS_KEY, stored, &size);
    nvs_close(handle);
    if (err != ESP_OK || size != sizeof(stored)) {
        memset(s_schedules, 0, sizeof(s_schedules));
        return;
    }
    for (int i = 0; i < BOARD_RELAY_COUNT; ++i) {
        if (!schedule_is_valid(&stored[i])) {
            memset(&stored[i], 0, sizeof(stored[i]));
        }
    }
    memcpy(s_schedules, stored, sizeof(s_schedules));
}

static void apply_relay_schedules(void)
{
    rv3028_datetime_t rtc;
    bool valid = false;
    if (rv3028_get_time(&rtc, &valid) != ESP_OK || !valid) return;

    relay_schedule_t schedules[BOARD_RELAY_COUNT];
    xSemaphoreTake(s_schedule_lock, portMAX_DELAY);
    memcpy(schedules, s_schedules, sizeof(schedules));
    xSemaphoreGive(s_schedule_lock);

    uint16_t now_minute = (uint16_t)rtc.hour * 60U + rtc.minute;
    for (int i = 0; i < BOARD_RELAY_COUNT; ++i) {
        relay_schedule_t *schedule = &schedules[i];
        if (!schedule->enabled) continue;
        uint16_t on_minute =
            (uint16_t)schedule->on_hour * 60U + schedule->on_minute;
        uint16_t off_minute =
            (uint16_t)schedule->off_hour * 60U + schedule->off_minute;
        if (on_minute == off_minute) continue;

        bool should_be_on = on_minute < off_minute
                                ? now_minute >= on_minute &&
                                      now_minute < off_minute
                                : now_minute >= on_minute ||
                                      now_minute < off_minute;
        if (relay_driver_get_commanded((uint8_t)i) != should_be_on) {
            esp_err_t err = relay_driver_set((uint8_t)i, should_be_on);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "Schedule set relay %d %s at %02u:%02u",
                         i + 1, should_be_on ? "ON" : "OFF",
                         rtc.hour, rtc.minute);
            }
        }
    }
}

static esp_err_t status_handler(httpd_req_t *req)
{
    uint16_t analog[BOARD_ANALOG_COUNT];
    uint16_t voltage[BOARD_ANALOG_COUNT];
    uint16_t current[BOARD_ANALOG_COUNT];
    unsigned modes[BOARD_ANALOG_COUNT];
    for (int i = 0; i < BOARD_ANALOG_COUNT; ++i) {
        uint32_t adc_mv = analog_input_get_adc_mv(i);
        analog[i] = analog_input_get_engineering_x100(i);
        /* 27k/10k divider factor=3.7; current uses a 100 ohm shunt. */
        voltage[i] = (uint16_t)((adc_mv * 37U + 50U) / 100U);
        current[i] = (uint16_t)((adc_mv * 37U + 5U) / 10U);
        modes[i] = (unsigned)analog_input_get_mode(i);
    }
    rv3028_datetime_t rtc = {0};
    bool rtc_valid = false;
    esp_err_t rtc_err = rv3028_get_time(&rtc, &rtc_valid);
    relay_schedule_t schedules[BOARD_RELAY_COUNT];
    xSemaphoreTake(s_schedule_lock, portMAX_DELAY);
    memcpy(schedules, s_schedules, sizeof(schedules));
    xSemaphoreGive(s_schedule_lock);

    char ssid[33] = "";
    wifi_config_t sta_cfg = {0};
    if (esp_wifi_get_config(WIFI_IF_STA, &sta_cfg) == ESP_OK) {
        memcpy(ssid, sta_cfg.sta.ssid, sizeof(sta_cfg.sta.ssid));
        ssid[sizeof(sta_cfg.sta.ssid)] = '\0';
    }

    char json[1024];
    snprintf(json, sizeof(json),
             "{\"relay_commands\":%u,\"relay_feedback\":%u,\"inputs\":%u,"
             "\"analog\":[%u,%u,%u,%u],\"modes\":[%u,%u,%u,%u],"
             "\"voltage\":[%u,%u,%u,%u],"
             "\"current\":[%u,%u,%u,%u],"
             "\"schedules\":[[%u,%u,%u,%u,%u],[%u,%u,%u,%u,%u],"
             "[%u,%u,%u,%u,%u],[%u,%u,%u,%u,%u]],"
             "\"rtc\":{\"valid\":%s,\"year\":%u,\"month\":%u,\"day\":%u,"
             "\"hour\":%u,\"minute\":%u,\"second\":%u},"
             "\"wifi\":{\"connected\":%s,\"ssid\":\"%s\","
             "\"ip\":\"" IPSTR "\"}}",
             relay_driver_get_commanded_mask(), relay_driver_get_mask(),
             digital_input_get_mask(),
             analog[0], analog[1], analog[2], analog[3],
             modes[0], modes[1], modes[2], modes[3],
             voltage[0], voltage[1], voltage[2], voltage[3],
             current[0], current[1], current[2], current[3],
             schedules[0].enabled, schedules[0].on_hour,
             schedules[0].on_minute, schedules[0].off_hour,
             schedules[0].off_minute,
             schedules[1].enabled, schedules[1].on_hour,
             schedules[1].on_minute, schedules[1].off_hour,
             schedules[1].off_minute,
             schedules[2].enabled, schedules[2].on_hour,
             schedules[2].on_minute, schedules[2].off_hour,
             schedules[2].off_minute,
             schedules[3].enabled, schedules[3].on_hour,
             schedules[3].on_minute, schedules[3].off_hour,
             schedules[3].off_minute,
             rtc_err == ESP_OK && rtc_valid ? "true" : "false",
             rtc.year, rtc.month, rtc.date, rtc.hour, rtc.minute, rtc.second,
             s_sta_connected ? "true" : "false", ssid, IP2STR(&s_sta_ip));
    return send_json(req, json);
}

static esp_err_t relay_handler(httpd_req_t *req)
{
    char body[WEB_BODY_MAX];
    int channel, on;
    if (receive_body(req, body, sizeof(body)) != ESP_OK ||
        !form_get_int(body, "channel", &channel) ||
        !form_get_int(body, "on", &on) ||
        channel < 0 || channel >= BOARD_RELAY_COUNT ||
        (on != 0 && on != 1)) {
        return send_error(req, "400 Bad Request", "继电器参数错误");
    }
    if (relay_driver_set((uint8_t)channel, on != 0) != ESP_OK) {
        return send_error(req, "500 Internal Server Error", "继电器控制失败");
    }
    return send_json(req, "{\"ok\":true}");
}

static esp_err_t time_handler(httpd_req_t *req)
{
    char body[WEB_BODY_MAX];
    int year, month, day, hour, minute, second;
    if (receive_body(req, body, sizeof(body)) != ESP_OK ||
        !form_get_int(body, "year", &year) ||
        !form_get_int(body, "month", &month) ||
        !form_get_int(body, "day", &day) ||
        !form_get_int(body, "hour", &hour) ||
        !form_get_int(body, "minute", &minute) ||
        !form_get_int(body, "second", &second)) {
        return send_error(req, "400 Bad Request", "时间参数错误");
    }
    rv3028_datetime_t rtc = {
        .year = (uint16_t)year, .month = (uint8_t)month,
        .date = (uint8_t)day, .weekday = 0, .hour = (uint8_t)hour,
        .minute = (uint8_t)minute, .second = (uint8_t)second,
    };
    if (rv3028_set_time(&rtc) != ESP_OK) {
        return send_error(req, "400 Bad Request", "日期无效或RTC写入失败");
    }
    return send_json(req, "{\"ok\":true}");
}

static esp_err_t schedule_handler(httpd_req_t *req)
{
    char body[WEB_BODY_MAX];
    int channel, enabled, on_hour, on_minute, off_hour, off_minute;
    if (receive_body(req, body, sizeof(body)) != ESP_OK ||
        !form_get_int(body, "channel", &channel) ||
        !form_get_int(body, "enabled", &enabled) ||
        !form_get_int(body, "on_hour", &on_hour) ||
        !form_get_int(body, "on_minute", &on_minute) ||
        !form_get_int(body, "off_hour", &off_hour) ||
        !form_get_int(body, "off_minute", &off_minute) ||
        channel < 0 || channel >= BOARD_RELAY_COUNT ||
        (enabled != 0 && enabled != 1) ||
        on_hour < 0 || on_hour > 23 || off_hour < 0 || off_hour > 23 ||
        on_minute < 0 || on_minute > 59 ||
        off_minute < 0 || off_minute > 59 ||
        (enabled && on_hour == off_hour && on_minute == off_minute)) {
        return send_error(req, "400 Bad Request",
                          "定时时间无效，开关时间不能相同");
    }

    relay_schedule_t updated[BOARD_RELAY_COUNT];
    xSemaphoreTake(s_schedule_lock, portMAX_DELAY);
    memcpy(updated, s_schedules, sizeof(updated));
    xSemaphoreGive(s_schedule_lock);
    updated[channel] = (relay_schedule_t) {
        .enabled = (uint8_t)enabled,
        .on_hour = (uint8_t)on_hour,
        .on_minute = (uint8_t)on_minute,
        .off_hour = (uint8_t)off_hour,
        .off_minute = (uint8_t)off_minute,
    };

    esp_err_t err = save_schedules(updated);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to save relay schedules: %s",
                 esp_err_to_name(err));
        return send_error(req, "500 Internal Server Error", "定时保存失败");
    }
    xSemaphoreTake(s_schedule_lock, portMAX_DELAY);
    memcpy(s_schedules, updated, sizeof(s_schedules));
    xSemaphoreGive(s_schedule_lock);
    ESP_LOGI(TAG, "Relay %d schedule: %s, ON %02d:%02d, OFF %02d:%02d",
             channel + 1, enabled ? "enabled" : "disabled",
             on_hour, on_minute, off_hour, off_minute);
    apply_relay_schedules();
    return send_json(req, "{\"ok\":true}");
}

static esp_err_t save_wifi_credentials(const char *ssid, const char *password)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(WEB_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_str(handle, "ssid", ssid);
    if (err == ESP_OK) err = nvs_set_str(handle, "password", password);
    if (err == ESP_OK) err = nvs_commit(handle);
    nvs_close(handle);
    return err;
}

static bool load_wifi_credentials(char *ssid, size_t ssid_size,
                                  char *password, size_t password_size)
{
    nvs_handle_t handle;
    if (nvs_open(WEB_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
        return false;
    }
    size_t ssid_len = ssid_size;
    size_t pass_len = password_size;
    esp_err_t err = nvs_get_str(handle, "ssid", ssid, &ssid_len);
    if (err == ESP_OK) {
        err = nvs_get_str(handle, "password", password, &pass_len);
    }
    nvs_close(handle);
    return err == ESP_OK && ssid[0];
}

static esp_err_t apply_sta_config(const char *ssid, const char *password)
{
    wifi_config_t cfg = {0};
    copy_wifi_field(cfg.sta.ssid, sizeof(cfg.sta.ssid), ssid);
    copy_wifi_field(cfg.sta.password, sizeof(cfg.sta.password), password);
    cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    cfg.sta.pmf_cfg.capable = true;
    cfg.sta.pmf_cfg.required = false;
    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    if (err == ESP_OK) {
        s_sta_has_credentials = true;
        s_sta_connected = false;
        s_sta_ip.addr = 0;
        esp_wifi_disconnect();
        err = esp_wifi_connect();
    }
    return err;
}

static esp_err_t wifi_handler(httpd_req_t *req)
{
    char body[WEB_BODY_MAX], ssid[33], password[65];
    if (receive_body(req, body, sizeof(body)) != ESP_OK ||
        !form_get(body, "ssid", ssid, sizeof(ssid)) ||
        !form_get(body, "password", password, sizeof(password)) ||
        !ssid[0]) {
        return send_error(req, "400 Bad Request", "Wi-Fi名称或密码无效");
    }
    esp_err_t err = save_wifi_credentials(ssid, password);
    if (err == ESP_OK) err = apply_sta_config(ssid, password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Wi-Fi configuration failed: %s",
                 esp_err_to_name(err));
        return send_error(req, "500 Internal Server Error", "Wi-Fi保存或连接失败");
    }
    ESP_LOGI(TAG, "Saved Wi-Fi SSID '%s'; connection started", ssid);
    return send_json(req, "{\"ok\":true}");
}

static void wifi_event_handler(void *argument, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)argument;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START &&
        s_sta_has_credentials) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_sta_connected = false;
        s_sta_ip.addr = 0;
        if (s_sta_has_credentials) esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *event = event_data;
        s_sta_ip = event->ip_info.ip;
        s_sta_connected = true;
        ESP_LOGI(TAG, "Connected to Wi-Fi, IP=" IPSTR, IP2STR(&s_sta_ip));
    }
}

static esp_err_t start_http_server(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 6144;
    /* Reclaim stale browser sockets after Wi-Fi interruptions. */
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 5;
    config.send_wait_timeout = 5;
    esp_err_t err = httpd_start(&s_server, &config);
    if (err != ESP_OK) return err;
    const httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = index_handler},
        {.uri = "/api/status", .method = HTTP_GET, .handler = status_handler},
        {.uri = "/api/relay", .method = HTTP_POST, .handler = relay_handler},
        {.uri = "/api/schedule", .method = HTTP_POST,
         .handler = schedule_handler},
        {.uri = "/api/time", .method = HTTP_POST, .handler = time_handler},
        {.uri = "/api/wifi", .method = HTTP_POST, .handler = wifi_handler},
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i) {
        err = httpd_register_uri_handler(s_server, &routes[i]);
        if (err != ESP_OK) return err;
    }
    return ESP_OK;
}

static esp_err_t init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        err = nvs_flash_erase();
        if (err == ESP_OK) err = nvs_flash_init();
    }
    return err;
}

static esp_err_t start_web_mode(void)
{
    ESP_LOGI(TAG, "SW3 held low for %d ms: starting web mode",
             WEB_SWITCH_HOLD_MS);
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) return err;
    err = esp_event_loop_create_default();
    if (err != ESP_OK) return err;
    s_ap_netif = esp_netif_create_default_wifi_ap();
    s_sta_netif = esp_netif_create_default_wifi_sta();
    if (!s_ap_netif || !s_sta_netif) {
        return ESP_ERR_NO_MEM;
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    err = esp_wifi_init(&init_cfg);
    if (err != ESP_OK) return err;
    err = esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                     wifi_event_handler, NULL);
    if (err == ESP_OK) {
        err = esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                         wifi_event_handler, NULL);
    }
    if (err != ESP_OK) return err;

    wifi_config_t ap_cfg = {0};
    snprintf((char *)ap_cfg.ap.ssid, sizeof(ap_cfg.ap.ssid), "%s",
             BOARD_WEB_AP_SSID);
    snprintf((char *)ap_cfg.ap.password, sizeof(ap_cfg.ap.password), "%s",
             BOARD_WEB_AP_PASSWORD);
    ap_cfg.ap.ssid_len = strlen(BOARD_WEB_AP_SSID);
    ap_cfg.ap.channel = 1;
    ap_cfg.ap.max_connection = 4;
    ap_cfg.ap.authmode = WIFI_AUTH_WPA2_PSK;

    char sta_ssid[33] = "", sta_password[65] = "";
    s_sta_has_credentials = load_wifi_credentials(
        sta_ssid, sizeof(sta_ssid), sta_password, sizeof(sta_password));
    err = esp_wifi_set_mode(s_sta_has_credentials
                                ? WIFI_MODE_APSTA : WIFI_MODE_AP);
    if (err == ESP_OK) err = esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    if (err == ESP_OK && s_sta_has_credentials) {
        wifi_config_t sta_cfg = {0};
        copy_wifi_field(sta_cfg.sta.ssid, sizeof(sta_cfg.sta.ssid),
                        sta_ssid);
        copy_wifi_field(sta_cfg.sta.password, sizeof(sta_cfg.sta.password),
                        sta_password);
        sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
        sta_cfg.sta.pmf_cfg.capable = true;
        err = esp_wifi_set_config(WIFI_IF_STA, &sta_cfg);
    }
    if (err == ESP_OK) err = esp_wifi_start();
    if (err != ESP_OK) return err;
    err = start_http_server();
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "AP='%s', password='%s', open http://192.168.4.1",
             BOARD_WEB_AP_SSID, BOARD_WEB_AP_PASSWORD);
    return ESP_OK;
}

static esp_err_t stop_web_mode(void)
{
    ESP_LOGI(TAG, "SW3 held low for %d ms: stopping web mode",
             WEB_SWITCH_HOLD_MS);
    esp_err_t result = ESP_OK;

    if (s_server) {
        esp_err_t err = httpd_stop(s_server);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "HTTP server stop failed: %s",
                     esp_err_to_name(err));
            result = err;
        }
        s_server = NULL;
    }

    esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                 wifi_event_handler);
    esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                 wifi_event_handler);

    esp_err_t err = esp_wifi_stop();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_INIT) {
        ESP_LOGW(TAG, "Wi-Fi stop failed: %s", esp_err_to_name(err));
        if (result == ESP_OK) result = err;
    }
    err = esp_wifi_deinit();
    if (err != ESP_OK && err != ESP_ERR_WIFI_NOT_INIT) {
        ESP_LOGW(TAG, "Wi-Fi deinit failed: %s", esp_err_to_name(err));
        if (result == ESP_OK) result = err;
    }

    if (s_sta_netif) {
        esp_netif_destroy_default_wifi(s_sta_netif);
        s_sta_netif = NULL;
    }
    if (s_ap_netif) {
        esp_netif_destroy_default_wifi(s_ap_netif);
        s_ap_netif = NULL;
    }
    err = esp_event_loop_delete_default();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Default event loop delete failed: %s",
                 esp_err_to_name(err));
        if (result == ESP_OK) result = err;
    }

    s_sta_has_credentials = false;
    s_sta_connected = false;
    s_sta_ip.addr = 0;
    ESP_LOGI(TAG, "Wi-Fi hotspot and web server stopped");
    return result;
}

esp_err_t web_config_start_if_requested(void)
{
    gpio_config_t sw3_cfg = {
        .pin_bit_mask = 1ULL << BOARD_WEB_CONFIG_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&sw3_cfg);
    if (err != ESP_OK) return err;
    err = init_nvs();
    if (err != ESP_OK) return err;
    s_schedule_lock = xSemaphoreCreateMutex();
    if (!s_schedule_lock) return ESP_ERR_NO_MEM;
    load_schedules();

    s_switch_initialized = true;
    s_switch_low = false;
    s_switch_raw_low = gpio_get_level(BOARD_WEB_CONFIG_GPIO) ==
                       BOARD_WEB_CONFIG_ACTIVE_LEVEL;
    s_switch_raw_changed_at = xTaskGetTickCount();
    s_switch_action_done = false;
    s_web_started = false;
    ESP_LOGI(TAG,
             "SW3 runtime detection ready: GPIO%d, debounce=%d ms, "
             "hold low for %d ms",
             BOARD_WEB_CONFIG_GPIO, WEB_SWITCH_DEBOUNCE_MS,
             WEB_SWITCH_HOLD_MS);
    return ESP_OK;
}

void web_config_poll(void)
{
    if (!s_switch_initialized) return;
    TickType_t now = xTaskGetTickCount();
    if (now - s_last_schedule_check >=
        pdMS_TO_TICKS(WEB_SCHEDULE_CHECK_MS)) {
        s_last_schedule_check = now;
        apply_relay_schedules();
    }
    bool raw_low = gpio_get_level(BOARD_WEB_CONFIG_GPIO) ==
                   BOARD_WEB_CONFIG_ACTIVE_LEVEL;
    if (raw_low != s_switch_raw_low) {
        s_switch_raw_low = raw_low;
        s_switch_raw_changed_at = now;
    }

    if (s_switch_raw_low != s_switch_low &&
        now - s_switch_raw_changed_at >=
            pdMS_TO_TICKS(WEB_SWITCH_DEBOUNCE_MS)) {
        s_switch_low = s_switch_raw_low;
        if (s_switch_low) {
            s_switch_action_done = false;
            s_switch_low_since = now;
            ESP_LOGI(TAG,
                     "SW3 debounced low; hold %d seconds to %s hotspot",
                     WEB_SWITCH_HOLD_MS / 1000,
                     s_web_started ? "stop" : "start");
        } else {
            ESP_LOGI(TAG, "SW3 debounced release; next long press is armed");
            s_switch_action_done = false;
        }
    }

    if (!s_switch_low) {
        return;
    }

    if (s_switch_action_done) return;
    if (now - s_switch_low_since < pdMS_TO_TICKS(WEB_SWITCH_HOLD_MS)) {
        return;
    }

    esp_err_t err = s_web_started ? stop_web_mode() : start_web_mode();
    if (err == ESP_OK) {
        s_web_started = !s_web_started;
    } else {
        ESP_LOGE(TAG, "Unable to toggle web mode: %s",
                 esp_err_to_name(err));
    }
    s_switch_action_done = true;
}
