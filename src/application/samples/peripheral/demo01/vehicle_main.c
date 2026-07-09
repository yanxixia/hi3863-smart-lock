/**
 * 智能车锁 - 车载端
 *
 * 硬件: 小熊派H3863 (WS63 / Hi3863V100)
 * GPIO: LED=2 蜂鸣器=8 舵机=5(PWM5) 振动=13(上拉,下降沿中断)
 *
 * 通信方案:
 *   - WiFi STA (车载端接收端): 连接移动AP Qian_Ru_Shi_Da_Sai/DUI, DHCP获取IP
 *   - UDP: 8081 收移动端 LOCK/UNLOCK, 每500ms单播 STATE:x,ALARM:y 到车载IP:8082
 *
 * 舵机 (SG90): PWM5, 50Hz, 上锁=1000us, 中位=1500us, 开锁=2000us
 *
 * 振动检测 (SW-18010P): 下降沿中断, 仅上锁态响应, 2秒窗口3次脉冲→报警
 *
 * 上电自检: LED闪3次→舵机演示3轮→蜂鸣器150ms
 *
 * Copyright (c) 2024. All rights reserved.
 */

#include <stdio.h>
#include <string.h>
#include "securec.h"
#include "common_def.h"
#include "soc_osal.h"
#include "app_init.h"
#include "cmsis_os2.h"
#include "gpio.h"
#include "pinctrl.h"
#include "pwm.h"
#include "tcxo.h"
#include "watchdog.h"
#include "wifi_device.h"
#include "wifi_event.h"
#include "wifi_linked_info.h"
#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "lwip/sockets.h"
#include "lwip/ip4_addr.h"
#include "lwip/inet.h"
#include "../../../../../vendor/BearPi-Pico_H3863/products/sle_uart/sle_uart_server/sle_uart_server.h"
#include "../../../../../vendor/BearPi-Pico_H3863/products/sle_uart/sle_uart_server/sle_uart_server.c"
#include "../../../../../vendor/BearPi-Pico_H3863/products/sle_uart/sle_uart_server/sle_uart_server_adv.c"

/* ==================== 宏定义 - GPIO ==================== */

#define PIN_LED                 2
#define PIN_BUZZER              8
#define PIN_SERVO               5
#define PIN_VIBRATION           13
#define SERVO_PWM_CHANNEL       5
#define SERVO_PIN_MODE          1

/* ==================== 宏定义 - 舵机 (SG90) ==================== */

#define SERVO_LOCK_PULSE_US        1000
#define SERVO_CENTER_PULSE_US      1500
#define SERVO_UNLOCK_PULSE_US      2000
#define SERVO_DEMO_PULSE_US        SERVO_UNLOCK_PULSE_US
#define SERVO_PERIOD_US            20000

/* ==================== 宏定义 - 上电自检 ==================== */

#define BOOT_BLINK_COUNT           3
#define BOOT_BLINK_DURATION_MS     250
#define BOOT_BEEP_DURATION_MS      150

/* ==================== 宏定义 - 振动检测 ==================== */

#define VIB_THRESHOLD              1
#define VIB_WINDOW_MS              500
#define VIB_DEBOUNCE_MS            10

/* ==================== 宏定义 - 报警蜂鸣 ==================== */

#define ALARM_ON_DURATION_MS       200
#define ALARM_OFF_DURATION_MS      200

/* ==================== 宏定义 - 通信 ==================== */

#define STATE_SEND_INTERVAL_MS     500
#define LINK_EVAL_INTERVAL_MS      1000
#define LINK_CONNECT_SUCCESS_COUNT 1
#define LINK_DISCONNECT_FAIL_COUNT 5

/* ==================== 宏定义 - WiFi STA ==================== */

#define WIFI_TARGET_SSID           "Qian_Ru_Shi_Da_Sai"
#define WIFI_TARGET_PASSWORD       "DUI"
#define MOBILE_AP_IP_OCT1          192
#define MOBILE_AP_IP_OCT2          168
#define MOBILE_AP_IP_OCT3          43
#define MOBILE_AP_IP_OCT4          1
#define WIFI_SCAN_AP_LIMIT         64
#define WIFI_GET_IP_MAX_COUNT      300
#define WIFI_CONNECT_SCAN_ROUNDS   5
#define UDP_LISTEN_PORT            8081
#define UDP_MOBILE_PORT            8082

/* ==================== 宏定义 - 看门狗 ==================== */

#define WDT_TIMEOUT_SEC            30

/* ==================== 类型定义 ==================== */

typedef enum {
    LOCK_STATE_UNLOCK = 0,
    LOCK_STATE_LOCKED = 1,
} lock_state_t;

typedef enum {
    ALARM_STATE_NORMAL = 0,
    ALARM_STATE_ACTIVE = 1,
} alarm_state_t;

enum wifi_sta_state_enum {
    WIFI_STA_STATE_INIT = 0,
    WIFI_STA_STATE_SCANNING,
    WIFI_STA_STATE_SCAN_DONE,
    WIFI_STA_STATE_FOUND_TARGET,
    WIFI_STA_STATE_CONNECTING,
    WIFI_STA_STATE_LINKED,
    WIFI_STA_STATE_GET_IP,
};

/* ==================== 全局变量 - 系统状态 ==================== */

static volatile lock_state_t  g_locked           = LOCK_STATE_LOCKED;
static volatile alarm_state_t g_alarm            = ALARM_STATE_NORMAL;
static volatile uint8_t       g_boot_done        = 0;
static volatile uint8_t       g_watchdog_ready   = 0;
static volatile uint8_t       g_alarm_event_pending = 0;
static volatile uint8_t       g_vib_irq_latched     = 0;

/* ==================== 全局变量 - WiFi ==================== */

static volatile uint8_t       g_wifi_state       = WIFI_STA_STATE_INIT;
static volatile uint8_t       g_wifi_ok          = 0;
static volatile uint8_t       g_sle_ok           = 0;
static volatile uint8_t       g_wifi_link_ok     = 0;
static volatile uint8_t       g_sle_link_ok      = 0;
static uint32_t               g_mobile_ap_ip     = 0;
static volatile int32_t       g_wifi_rssi        = -127;
static volatile uint8_t       g_state_report_now = 0;
static volatile uint8_t       g_wifi_rx_in_window = 0;
static volatile uint8_t       g_sle_rx_in_window  = 0;
static volatile uint8_t       g_wifi_success_streak = 0;
static volatile uint8_t       g_wifi_fail_streak    = 0;
static volatile uint8_t       g_sle_success_streak  = 0;
static volatile uint8_t       g_sle_fail_streak     = 0;
static volatile uint32_t      g_wifi_window_elapsed_ms = 0;
static volatile uint32_t      g_sle_window_elapsed_ms  = 0;

/* ==================== 全局变量 - UDP ==================== */

static int                    g_udp_socket       = -1;

/* ==================== 全局变量 - 舵机 ==================== */

static uint32_t               g_pwm_clock_freq   = 32000000;
static volatile uint32_t      g_servo_target_pulse_us = SERVO_CENTER_PULSE_US;

static void vehicle_send_inbox_event(const char *event_text);
static int vehicle_format_state_message(char *message, size_t message_size);
static void vehicle_send_message_all(const char *message);
static void vehicle_latch_alarm_from_isr(void);
static uint8_t vehicle_vibration_detection_enabled(void);

static uint32_t vehicle_get_mobile_ap_ip(void)
{
    ip4_addr_t mobile_ip;
    IP4_ADDR(&mobile_ip, MOBILE_AP_IP_OCT1, MOBILE_AP_IP_OCT2,
             MOBILE_AP_IP_OCT3, MOBILE_AP_IP_OCT4);
    return mobile_ip.addr;
}

static void vehicle_watchdog_kick_safe(void)
{
    if (g_watchdog_ready) {
        uapi_watchdog_kick();
    }
}

static void vehicle_send_state_now(void)
{
    char state_message[64];
    int message_len = vehicle_format_state_message(state_message,
                                                   sizeof(state_message));
    if (message_len > 0) {
        vehicle_send_message_all(state_message);
    }
}

/* ==================== 振动传感器 ISR ==================== */

static void vibration_isr_handler(pin_t pin, uintptr_t param)
{
    UNUSED(pin);
    UNUSED(param);

    if (g_vib_irq_latched) {
        return;
    }
    if (!vehicle_vibration_detection_enabled()) {
        return;
    }
    if (uapi_gpio_get_val(PIN_VIBRATION) != GPIO_LEVEL_HIGH) {
        return;
    }

    g_vib_irq_latched = 1;
    vehicle_latch_alarm_from_isr();
}

static void vehicle_set_lock_state(lock_state_t new_state, const char *reason)
{
    if (g_locked == new_state) {
        return;
    }

    g_locked = new_state;
    g_state_report_now = 1;
    printf("[LOCK] %s -> %s\r\n",
           (reason != NULL) ? reason : "update",
           (new_state == LOCK_STATE_LOCKED) ? "LOCKED" : "UNLOCKED");
}

static void vehicle_latch_alarm_from_isr(void)
{
    if (g_alarm == ALARM_STATE_ACTIVE) {
        return;
    }

    g_alarm = ALARM_STATE_ACTIVE;
    g_locked = LOCK_STATE_LOCKED;
    g_servo_target_pulse_us = SERVO_LOCK_PULSE_US;
    g_state_report_now = 1;
    g_alarm_event_pending = 1;
    uapi_gpio_set_val(PIN_BUZZER, GPIO_LEVEL_HIGH);
}

static void vehicle_note_link_rx(uint8_t from_sle)
{
    if (from_sle) {
        g_sle_rx_in_window = 1;
    } else {
        g_wifi_rx_in_window = 1;
    }
}

static void vehicle_handle_wifi_link_sample(uint8_t success)
{
    if (success) {
        if (g_wifi_success_streak < 0xFF) {
            g_wifi_success_streak++;
        }
        g_wifi_fail_streak = 0;

        if (!g_wifi_link_ok &&
            (g_wifi_success_streak >= LINK_CONNECT_SUCCESS_COUNT)) {
            g_wifi_link_ok = 1;
            printf("[WIFI] peer connected\r\n");
        }
        return;
    }

    g_wifi_success_streak = 0;
    if (g_wifi_fail_streak < 0xFF) {
        g_wifi_fail_streak++;
    }

    if (g_wifi_link_ok &&
        (g_wifi_fail_streak >= LINK_DISCONNECT_FAIL_COUNT)) {
        g_wifi_link_ok = 0;
        printf("[WIFI] peer disconnected\r\n");
    }
}

static void vehicle_handle_sle_link_sample(uint8_t success)
{
    if (success) {
        if (g_sle_success_streak < 0xFF) {
            g_sle_success_streak++;
        }
        g_sle_fail_streak = 0;

        if (!g_sle_link_ok &&
            (g_sle_success_streak >= LINK_CONNECT_SUCCESS_COUNT)) {
            g_sle_link_ok = 1;
            printf("[SLE] peer connected\r\n");
            if (g_alarm != ALARM_STATE_ACTIVE) {
                vehicle_set_lock_state(LOCK_STATE_UNLOCK,
                                       "SLE link confirmed");
            } else {
                printf("[SLE] alarm active, keep locked\r\n");
            }
        }
        return;
    }

    g_sle_success_streak = 0;
    if (g_sle_fail_streak < 0xFF) {
        g_sle_fail_streak++;
    }

    if (g_sle_link_ok &&
        (g_sle_fail_streak >= LINK_DISCONNECT_FAIL_COUNT)) {
        g_sle_link_ok = 0;
        printf("[SLE] peer disconnected\r\n");
        vehicle_set_lock_state(LOCK_STATE_LOCKED, "SLE link lost");
    }
}

static void vehicle_link_monitor_update(uint32_t elapsed_ms,
                                        uint8_t sle_transport_connected)
{
    g_wifi_window_elapsed_ms += elapsed_ms;
    while (g_wifi_window_elapsed_ms >= LINK_EVAL_INTERVAL_MS) {
        g_wifi_window_elapsed_ms -= LINK_EVAL_INTERVAL_MS;
        vehicle_handle_wifi_link_sample((uint8_t)(g_wifi_ok &&
                                                  g_wifi_rx_in_window));
        g_wifi_rx_in_window = 0;
    }

    g_sle_window_elapsed_ms += elapsed_ms;
    while (g_sle_window_elapsed_ms >= LINK_EVAL_INTERVAL_MS) {
        g_sle_window_elapsed_ms -= LINK_EVAL_INTERVAL_MS;
        vehicle_handle_sle_link_sample((uint8_t)(sle_transport_connected ||
                                                 g_sle_rx_in_window));
        g_sle_rx_in_window = 0;
    }
}

static uint8_t vehicle_vibration_detection_enabled(void)
{
    return (uint8_t)((g_locked == LOCK_STATE_LOCKED) &&
                     (g_alarm == ALARM_STATE_NORMAL));
}

/* ==================== 舵机驱动 ==================== */

static void servo_pwm_initialize(void)
{
    printf("[SERVO] PWM init pin=%d mode=%d ch=%d\r\n",
           PIN_SERVO, SERVO_PIN_MODE, SERVO_PWM_CHANNEL);
    uapi_pin_set_mode(PIN_SERVO, SERVO_PIN_MODE);
    uapi_pwm_init();
    g_pwm_clock_freq = uapi_pwm_get_frequency(SERVO_PWM_CHANNEL);
    if (g_pwm_clock_freq == 0) {
        g_pwm_clock_freq = 32000000;
    }
    printf("[SERVO] PWM clock=%lu\r\n", (unsigned long)g_pwm_clock_freq);
}

static void servo_set_pulse_width(uint32_t pulse_width_us)
{
    static uint32_t last_pulse_width_us = 0;
    uint64_t high_clock_counts;
    uint64_t low_clock_counts;
    pwm_config_t pwm_config;

    if (pulse_width_us > SERVO_PERIOD_US) {
        pulse_width_us = SERVO_PERIOD_US;
    }
    if (pulse_width_us < 500) {
        pulse_width_us = 500;
    }

    g_servo_target_pulse_us = pulse_width_us;
    if (last_pulse_width_us != pulse_width_us) {
        last_pulse_width_us = pulse_width_us;
        printf("[SERVO] target pulse=%luus\r\n", (unsigned long)pulse_width_us);
    }

    high_clock_counts = (uint64_t)pulse_width_us * g_pwm_clock_freq / 1000000ULL;
    low_clock_counts = (uint64_t)(SERVO_PERIOD_US - pulse_width_us) * g_pwm_clock_freq / 1000000ULL;
    if (high_clock_counts == 0) {
        high_clock_counts = 1;
    }
    if (low_clock_counts == 0) {
        low_clock_counts = 1;
    }

    pwm_config.low_time = (uint32_t)low_clock_counts;
    pwm_config.high_time = (uint32_t)high_clock_counts;
    pwm_config.offset_time = 0;
    pwm_config.cycles = 0xFF;
    pwm_config.repeat = true;

    if (uapi_pwm_update_cfg(SERVO_PWM_CHANNEL, &pwm_config) != ERRCODE_SUCC) {
        (void)uapi_pwm_close(SERVO_PWM_CHANNEL);
        (void)uapi_pwm_open(SERVO_PWM_CHANNEL, &pwm_config);
        (void)uapi_pwm_start(SERVO_PWM_CHANNEL);
    }
}

static void servo_demonstration_run(void)
{
    int demo_round;
    int pulse_index;

    printf("[SERVO] demo start lock=%d center=%d unlock=%d\r\n",
           SERVO_LOCK_PULSE_US, SERVO_CENTER_PULSE_US, SERVO_UNLOCK_PULSE_US);
    for (demo_round = 0; demo_round < BOOT_BLINK_COUNT; demo_round++) {
        UNUSED(pulse_index);
        servo_set_pulse_width(SERVO_DEMO_PULSE_US);
        osal_msleep(600);
        servo_set_pulse_width(SERVO_CENTER_PULSE_US);
        osal_msleep(600);
        servo_set_pulse_width(SERVO_LOCK_PULSE_US);
        osal_msleep(600);
    }
    printf("[SERVO] demo done\r\n");
}

static void servo_control_thread(void *arg)
{
    UNUSED(arg);

    while (!g_boot_done) {
        osal_msleep(10);
    }

    while (1) {
        servo_set_pulse_width(g_servo_target_pulse_us);
        osal_msleep(50);
    }
}

static void vehicle_apply_remote_command(const char *command)
{
    if (command == NULL) {
        return;
    }

    if (strcmp(command, "UNLOCK") == 0) {
        if (g_alarm == ALARM_STATE_ACTIVE) {
            printf("[CTRL] UNLOCK ignored, alarm active\r\n");
            return;
        }
        vehicle_set_lock_state(LOCK_STATE_UNLOCK, "remote command");
        printf("[CTRL] UNLOCK\r\n");
    } else if (strcmp(command, "LOCK") == 0) {
        vehicle_set_lock_state(LOCK_STATE_LOCKED, "remote command");
        printf("[CTRL] LOCK\r\n");
    } else if ((strcmp(command, "PING:WIFI") == 0) ||
               (strcmp(command, "PING:SLE") == 0)) {
        printf("[HB] RX %s\r\n", command);
    }
}

static int vehicle_format_state_message(char *message, size_t message_size)
{
    return snprintf_s(message, message_size, message_size - 1,
                      "STATE:%d,ALARM:%d,RSSI:%d",
                      (int)g_locked, (int)g_alarm, (int)g_wifi_rssi);
}

static void vehicle_send_message_all(const char *message)
{
    if (message == NULL) {
        return;
    }

    if ((g_mobile_ap_ip != 0) && (g_udp_socket >= 0)) {
        struct sockaddr_in dest_address = {0};
        dest_address.sin_family      = AF_INET;
        dest_address.sin_port        = htons(UDP_MOBILE_PORT);
        dest_address.sin_addr.s_addr = g_mobile_ap_ip;
        (void)sendto(g_udp_socket, message, strlen(message), 0,
                     (struct sockaddr *)&dest_address, sizeof(dest_address));
    }

    if (g_sle_ok && sle_uart_client_is_connected()) {
        (void)sle_uart_server_send_report_by_handle((const uint8_t *)message,
                                                    (uint16_t)strlen(message));
    }
}

static void vehicle_send_inbox_event(const char *event_text)
{
    char event_message[48];

    if (event_text == NULL) {
        return;
    }

    if (snprintf_s(event_message, sizeof(event_message),
                   sizeof(event_message) - 1,
                   "INBOX:%s", event_text) <= 0) {
        return;
    }
    vehicle_send_message_all(event_message);
}

static void vehicle_sle_message_cb(uint8_t *buffer_addr, uint16_t buffer_size)
{
    UNUSED(buffer_size);

    if (buffer_addr == NULL) {
        return;
    }
    printf("[SLE] EVENT %s\r\n", buffer_addr);
}

static void vehicle_sle_read_request_cbk(uint8_t server_id,
                                         uint16_t conn_id,
                                         ssaps_req_read_cb_t *read_cb_para,
                                         errcode_t status)
{
    UNUSED(server_id);
    UNUSED(conn_id);
    UNUSED(read_cb_para);
    UNUSED(status);
}

static void vehicle_sle_write_request_cbk(uint8_t server_id,
                                          uint16_t conn_id,
                                          ssaps_req_write_cb_t *write_cb_para,
                                          errcode_t status)
{
    UNUSED(server_id);
    UNUSED(conn_id);

    if ((status != ERRCODE_SUCC) || (write_cb_para == NULL) ||
        (write_cb_para->value == NULL) || (write_cb_para->length == 0)) {
        return;
    }

    char command_buffer[32];
    uint16_t command_length = write_cb_para->length;
    if (command_length >= sizeof(command_buffer)) {
        command_length = sizeof(command_buffer) - 1;
    }

    if (memcpy_s(command_buffer, sizeof(command_buffer),
                 write_cb_para->value, command_length) != EOK) {
        return;
    }
    command_buffer[command_length] = '\0';
    printf("[SLE] RX %s\r\n", command_buffer);
    vehicle_note_link_rx(1);
    vehicle_apply_remote_command(command_buffer);
}

/* ==================== WiFi STA 连接 ==================== */

static int wifi_sta_connect_and_get_ip(void)
{
    char interface_name[17]     = "wlan0";
    char target_ssid[]          = WIFI_TARGET_SSID;
    char target_key[]           = WIFI_TARGET_PASSWORD;
    wifi_sta_config_stru bss    = {0};
    struct netif *netif_ptr     = NULL;
    wifi_linked_info_stru status = {0};
    uint8_t link_retry;
    uint8_t scan_round;
    uint8_t connected = 0;

    printf("[STA] wait wifi init...\r\n");
    while (wifi_is_wifi_inited() == 0) {
        osal_msleep(100);
    }
    printf("[STA] wifi inited, wait 3s...\r\n");
    osal_msleep(3000);

    printf("[STA] enable\r\n");
    if (wifi_sta_enable() != 0) {
        printf("[STA] en fail\r\n");
        return -1;
    }
    printf("[STA] en ok\r\n");

    for (scan_round = 0; scan_round < WIFI_CONNECT_SCAN_ROUNDS; scan_round++) {
        printf("[STA] scan\r\n");
        if (wifi_sta_scan() != 0) {
            printf("[STA] scan fail, retry\r\n");
            osal_msleep(2000);
            continue;
        }
        osal_msleep(3000);

        {
            uint32_t n  = WIFI_SCAN_AP_LIMIT;
            uint32_t sz = sizeof(wifi_scan_info_stru) * n;
            wifi_scan_info_stru *r = osal_kmalloc(sz, OSAL_GFP_ATOMIC);
            if (!r) {
                osal_msleep(2000);
                continue;
            }
            (void)memset_s(r, sz, 0, sz);
            if (wifi_sta_get_scan_info(r, &n) != 0) {
                osal_kfree(r);
                printf("[STA] get scan info fail, retry\r\n");
                osal_msleep(2000);
                continue;
            }
            uint8_t found = 0;
            uint32_t i;
            for (i = 0; i < n; i++) {
                if (strlen(target_ssid) == strlen((char*)r[i].ssid) &&
                    memcmp(target_ssid, r[i].ssid, strlen(target_ssid)) == 0) {
                    (void)memcpy_s(bss.ssid, WIFI_MAX_SSID_LEN, r[i].ssid, strlen((char*)r[i].ssid));
                    (void)memcpy_s(bss.bssid, WIFI_MAC_LEN, r[i].bssid, WIFI_MAC_LEN);
                    bss.security_type = r[i].security_type;
                    if (r[i].security_type == WIFI_SEC_TYPE_OPEN) {
                        bss.pre_shared_key[0] = '\0';
                    } else {
                        (void)memcpy_s(bss.pre_shared_key, WIFI_MAX_KEY_LEN,
                                       target_key, strlen(target_key));
                    }
                    bss.ip_type = 1;
                    found = 1;
                    break;
                }
            }
            osal_kfree(r);
            if (!found) {
                printf("[STA] AP not found, retry\r\n");
                osal_msleep(2000);
                continue;
            }
        }
        printf("[STA] found, connecting\r\n");

        if (wifi_sta_connect(&bss) != 0) {
            printf("[STA] connect fail, retry\r\n");
            osal_msleep(2000);
            continue;
        }

        for (link_retry = 0; link_retry < 10; link_retry++) {
            osal_msleep(500);
            (void)memset_s(&status, sizeof(status), 0, sizeof(status));
            if (wifi_sta_get_ap_info(&status) != 0) {
                continue;
            }
            if (status.conn_state == 1) {
                g_wifi_rssi = status.rssi;
                connected = 1;
                break;
            }
        }
        if (connected) {
            break;
        }
        printf("[STA] link check fail, retry\r\n");
        osal_msleep(2000);
    }

    if (!connected) {
        printf("[STA] connect timeout\r\n");
        return -1;
    }

    printf("[STA] linked, DHCP\r\n");
    netif_ptr = netifapi_netif_find(interface_name);
    if (!netif_ptr || netifapi_dhcp_start(netif_ptr) != 0) {
        printf("[STA] DHCP start fail\r\n");
        return -1;
    }

    {
        uint8_t i;
        for (i = 0; i < 40; i++) {
            osal_msleep(500);
            if (netifapi_dhcp_is_bound(netif_ptr) == 0) { break; }
        }
    }
    {
        uint8_t i;
        for (i = 0; i < 10; i++) {
            osal_msleep(10);
            if (netif_ptr->ip_addr.u_addr.ip4.addr != 0) { break; }
        }
    }

    g_mobile_ap_ip = vehicle_get_mobile_ap_ip();
    g_wifi_rssi = status.rssi;
    printf("[STA] local ip=%u.%u.%u.%u\r\n",
           (netif_ptr->ip_addr.u_addr.ip4.addr & 0xFF),
           (netif_ptr->ip_addr.u_addr.ip4.addr >> 8) & 0xFF,
           (netif_ptr->ip_addr.u_addr.ip4.addr >> 16) & 0xFF,
           (netif_ptr->ip_addr.u_addr.ip4.addr >> 24) & 0xFF);
    printf("[STA] mobile ap ip=%u.%u.%u.%u\r\n",
           (g_mobile_ap_ip & 0xFF), (g_mobile_ap_ip >> 8) & 0xFF,
           (g_mobile_ap_ip >> 16) & 0xFF, (g_mobile_ap_ip >> 24) & 0xFF);
    return 0;
}

/* ==================== UDP 通信 ==================== */

static int vehicle_udp_initialize(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        return -1;
    }

    int broadcast_enable = 1;
    (void)setsockopt(sock, SOL_SOCKET, SO_BROADCAST,
                     (const void *)&broadcast_enable,
                     sizeof(broadcast_enable));

    struct sockaddr_in bind_address = {0};
    bind_address.sin_family      = AF_INET;
    bind_address.sin_port        = htons(UDP_LISTEN_PORT);
    bind_address.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr *)&bind_address,
             sizeof(bind_address)) < 0) {
        lwip_close(sock);
        return -1;
    }

    return sock;
}

static void vehicle_wifi_thread(void *arg)
{
    UNUSED(arg);
    wifi_linked_info_stru link_info = {0};

    while (!g_boot_done) {
        osal_msleep(100);
    }

    while (1) {
        if (!g_wifi_ok) {
            printf("[STA] Starting WiFi STA\r\n");
            if (wifi_sta_connect_and_get_ip() == 0) {
                g_wifi_ok = 1;
                g_state_report_now = 1;
                printf("[STA] WiFi OK\r\n");
            } else {
                g_wifi_rssi = -127;
                printf("[STA] WiFi FAIL, retry later\r\n");
            }
        }

        if (g_wifi_ok && (g_udp_socket < 0)) {
            g_udp_socket = vehicle_udp_initialize();
            if (g_udp_socket >= 0) {
                uapi_gpio_set_val(PIN_LED, GPIO_LEVEL_HIGH);
                printf("[UDP] socket ok\r\n");
            } else {
                printf("[UDP] socket fail\r\n");
            }
        }

        if (g_wifi_ok) {
            (void)memset_s(&link_info, sizeof(link_info), 0, sizeof(link_info));
            if ((wifi_sta_get_ap_info(&link_info) != 0) ||
                (link_info.conn_state != 1)) {
                g_wifi_ok = 0;
                g_wifi_rssi = -127;
                g_mobile_ap_ip = 0;
                g_state_report_now = 1;
                if (g_udp_socket >= 0) {
                    lwip_close(g_udp_socket);
                    g_udp_socket = -1;
                }
                uapi_gpio_set_val(PIN_LED, GPIO_LEVEL_LOW);
            } else {
                g_wifi_rssi = link_info.rssi;
            }
        }

        osal_msleep(g_wifi_ok ? 2000 : 3000);
    }
}

static void vehicle_sle_thread(void *arg)
{
    UNUSED(arg);

    while (!g_boot_done) {
        osal_msleep(100);
    }

    printf("[SLE] init server\r\n");
    sle_uart_server_register_msg(vehicle_sle_message_cb);
    if (sle_uart_server_init(vehicle_sle_read_request_cbk,
                             vehicle_sle_write_request_cbk) == ERRCODE_SUCC) {
        g_sle_ok = 1;
        printf("[SLE] server ready\r\n");
    } else {
        printf("[SLE] server init fail\r\n");
    }

    while (1) {
        osal_msleep(1000);
    }
}

/* ==================== 振动检测线程 ==================== */

static void vibration_monitor_thread(void *arg)
{
    UNUSED(arg);

    while (1) {
        osal_msleep(1000);
    }
}

/* ==================== 蜂鸣器线程 ==================== */

static void buzzer_control_thread(void *arg)
{
    UNUSED(arg);

    while (!g_boot_done) {
        osal_msleep(50);
    }

    while (1) {
        if (g_alarm == ALARM_STATE_ACTIVE) {
            uapi_gpio_set_val(PIN_BUZZER, GPIO_LEVEL_HIGH);
            osal_msleep(ALARM_ON_DURATION_MS);
            uapi_gpio_set_val(PIN_BUZZER, GPIO_LEVEL_LOW);
            osal_msleep(ALARM_OFF_DURATION_MS);
        } else {
            uapi_gpio_set_val(PIN_BUZZER, GPIO_LEVEL_LOW);
            osal_msleep(100);
        }
    }
}

/* ==================== 主控线程 ==================== */

static void vehicle_main_thread(void *arg)
{
    UNUSED(arg);

    /* ===== 第1步: 外设时钟初始化 ===== */
    uapi_tcxo_init();
    osal_msleep(10);

    /* ===== 第2步: GPIO 子系统初始化 ===== */
    uapi_pin_init();
    uapi_gpio_init();

    uapi_pin_set_mode(PIN_LED, PIN_MODE_0);
    uapi_gpio_set_dir(PIN_LED, GPIO_DIRECTION_OUTPUT);
    uapi_gpio_set_val(PIN_LED, GPIO_LEVEL_LOW);

    uapi_pin_set_mode(PIN_BUZZER, PIN_MODE_0);
    uapi_gpio_set_dir(PIN_BUZZER, GPIO_DIRECTION_OUTPUT);
    uapi_gpio_set_val(PIN_BUZZER, GPIO_LEVEL_LOW);

    uapi_pin_set_mode(PIN_SERVO, SERVO_PIN_MODE);

    uapi_pin_set_mode(PIN_VIBRATION, PIN_MODE_0);
    uapi_gpio_set_dir(PIN_VIBRATION, GPIO_DIRECTION_INPUT);
    uapi_pin_set_pull(PIN_VIBRATION, PIN_PULL_TYPE_DOWN);
    printf("[VIB] gpio=%d pull=DOWN init_level=%d\r\n",
           PIN_VIBRATION, (int)uapi_gpio_get_val(PIN_VIBRATION));
    servo_pwm_initialize();
    servo_set_pulse_width(SERVO_CENTER_PULSE_US);
    osal_msleep(200);

    /* ===== 第3步: 上电自检 ===== */

    int blink_index;
    for (blink_index = 0; blink_index < BOOT_BLINK_COUNT; blink_index++) {
        uapi_gpio_set_val(PIN_LED, GPIO_LEVEL_HIGH);
        osal_msleep(BOOT_BLINK_DURATION_MS);
        uapi_gpio_set_val(PIN_LED, GPIO_LEVEL_LOW);
        osal_msleep(BOOT_BLINK_DURATION_MS);
    }

    servo_demonstration_run();

    uapi_gpio_set_val(PIN_BUZZER, GPIO_LEVEL_HIGH);
    osal_msleep(BOOT_BEEP_DURATION_MS);
    uapi_gpio_set_val(PIN_BUZZER, GPIO_LEVEL_LOW);

    uapi_gpio_set_val(PIN_LED, GPIO_LEVEL_LOW);

    /* ===== 第4步: 设定舵机初始锁位 ===== */
    servo_set_pulse_width(SERVO_CENTER_PULSE_US);
    osal_msleep(300);
    servo_set_pulse_width(SERVO_LOCK_PULSE_US);

    /* ===== 第5步: 注册振动传感器中断 ===== */
    uapi_gpio_register_isr_func(PIN_VIBRATION,
                                 GPIO_INTERRUPT_RISING_EDGE,
                                 vibration_isr_handler);

    g_boot_done = 1;

    /* ===== 第6步: 看门狗 ===== */
    uapi_watchdog_init(WDT_TIMEOUT_SEC);
    uapi_watchdog_enable(WDT_MODE_RESET);
    g_watchdog_ready = 1;
    printf("[WDT] enabled\r\n");

    /* ===== 主循环: UDP 通信 + 舵机控制 + 状态广播 ===== */

    uint32_t last_broadcast_tick = 0;
    lock_state_t last_locked = g_locked;
    alarm_state_t last_alarm = g_alarm;

    while (1) {
        vehicle_watchdog_kick_safe();
        uint32_t current_tick = osKernelGetTickCount();
        uint8_t sle_transport_connected =
            (uint8_t)(sle_uart_client_is_connected() ? 1 : 0);

        /* --- 接收移动端 UDP 命令 --- */
        if (g_udp_socket >= 0) {
            char rx_buffer[128] = {0};
            struct sockaddr_in sender_address = {0};
            socklen_t sender_length = sizeof(sender_address);

            int received_len = recvfrom(g_udp_socket, rx_buffer,
                                        sizeof(rx_buffer) - 1,
                                        MSG_DONTWAIT,
                                        (struct sockaddr *)&sender_address,
                                        &sender_length);
            if (received_len > 0) {
                rx_buffer[received_len] = '\0';
                printf("[UDP] RX %s\r\n", rx_buffer);
                vehicle_note_link_rx(0);
                vehicle_apply_remote_command(rx_buffer);
            }
        }

        /* --- 更新舵机目标 --- */
        g_servo_target_pulse_us = (g_locked == LOCK_STATE_LOCKED) ?
                                  SERVO_LOCK_PULSE_US : SERVO_UNLOCK_PULSE_US;

        if ((g_locked != last_locked) || (g_alarm != last_alarm)) {
            g_state_report_now = 1;
            last_locked = g_locked;
            last_alarm = g_alarm;
        }

        if (g_alarm_event_pending) {
            g_alarm_event_pending = 0;
            vehicle_send_state_now();
            vehicle_send_inbox_event("VIBRATION ALERT");
        }

        /* --- UDP 单播状态到移动端 --- */
        if (g_state_report_now ||
            ((current_tick - last_broadcast_tick) >= STATE_SEND_INTERVAL_MS)) {
            last_broadcast_tick = current_tick;
            g_state_report_now = 0;

            char state_message[64];
            int message_len = vehicle_format_state_message(state_message,
                                                           sizeof(state_message));
            if (message_len > 0) {
                vehicle_send_message_all(state_message);
            }
        }

        vehicle_link_monitor_update(50, sle_transport_connected);

        osal_msleep(50);
    }
}

/* ==================== 系统入口 ==================== */

static void vehicle_application_entry(void)
{
    osThreadAttr_t thread_attrs = {0};

    thread_attrs.name       = "VehicleMain";
    thread_attrs.stack_size = 0x4000;
    thread_attrs.priority   = osPriorityNormal;
    (void)osThreadNew((osThreadFunc_t)vehicle_main_thread, NULL,
                      &thread_attrs);

    thread_attrs.name       = "VehicleVib";
    thread_attrs.stack_size = 0x1000;
    thread_attrs.priority   = osPriorityRealtime;
    (void)osThreadNew((osThreadFunc_t)vibration_monitor_thread, NULL,
                      &thread_attrs);

    thread_attrs.name       = "VehicleBuzzer";
    thread_attrs.stack_size = 0x1000;
    thread_attrs.priority   = osPriorityNormal;
    (void)osThreadNew((osThreadFunc_t)buzzer_control_thread, NULL,
                      &thread_attrs);

    thread_attrs.name       = "VehicleServo";
    thread_attrs.stack_size = 0x1000;
    thread_attrs.priority   = osPriorityNormal;
    (void)osThreadNew((osThreadFunc_t)servo_control_thread, NULL,
                      &thread_attrs);

    thread_attrs.name       = "VehicleWiFi";
    thread_attrs.stack_size = 0x2000;
    thread_attrs.priority   = osPriorityBelowNormal;
    (void)osThreadNew((osThreadFunc_t)vehicle_wifi_thread, NULL,
                      &thread_attrs);

    thread_attrs.name       = "VehicleSLE";
    thread_attrs.stack_size = 0x2000;
    thread_attrs.priority   = osPriorityBelowNormal;
    (void)osThreadNew((osThreadFunc_t)vehicle_sle_thread, NULL,
                      &thread_attrs);
}

app_run(vehicle_application_entry);
