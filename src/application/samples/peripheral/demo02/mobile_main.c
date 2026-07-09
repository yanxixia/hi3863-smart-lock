/**
 * 智能车锁 - 移动端
 *
 * 硬件: 小熊派H3863 (WS63)
 * GPIO: LED=2 蜂鸣器=8 按键=7(上拉,下降沿中断)
 * OLED: I2C1 GPIO15=SDA(PIN_MODE_2) GPIO16=SCL(PIN_MODE_2)
 *
 * 通信方案:
 *   - WiFi SoftAP (移动端发射端): Qian_Ru_Shi_Da_Sai/DUI, CH6, 192.168.43.1, DHCP Server
 *   - UDP: 8082 收车载端 STATE:x,ALARM:y 广播, 首次收到后记录源IP, 回发 LOCK/UNLOCK 到 8081
 *
 * 通信协议:
 *   - 车载→移动 (UDP): STATE:x,ALARM:y  (x=0开锁/1上锁, y=0正常/1报警)
 *   - 移动→车载 (UDP): UNLOCK / LOCK
 *
 * 按键 (500ms 点击窗口):
 *   - 非报警态: 单击→收件箱(ISR直接触发,2秒自动消失), 三击→UDP切锁
 *   - 报警态:   双击→取消本地报警
 *
 * OLED 显示 (4行 + RSSI信号条):
 *   - 第1行: WiFi状态 (WiFi Mode)
 *   - 第2行: 车锁状态 (Lock:LOCKED / Lock:UNLOCKED / Lock:Unknown)
 *   - 第3行: WiFi连接状态 (WiFi:OK / WiFi:NO)
 *   - 第4行: 振动报警状态 (Vibration:YES! / Vibration:NO / Vibration:Unknown)
 *   - 底部: RSSI信号强度横条(5格)
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
#include "i2c.h"
#include "watchdog.h"
#include "wifi_hotspot.h"
#include "wifi_hotspot_config.h"
#include "lwip/netif.h"
#include "lwip/netifapi.h"
#include "lwip/sockets.h"
#include "lwip/ip4_addr.h"
#include "lwip/inet.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "../../../../../vendor/BearPi-Pico_H3863/products/sle_uart/sle_uart_client/sle_uart_client.h"
#include "../../../../../vendor/BearPi-Pico_H3863/products/sle_uart/sle_uart_client/sle_uart_client.c"

/* ==================== 宏定义 - GPIO ==================== */

#define PIN_LED                 2
#define PIN_BUZZER              8
#define PIN_KEY                 7

/* ==================== 宏定义 - OLED ==================== */

#define PIN_OLED_SCL            16
#define PIN_OLED_SDA            15
#define OLED_I2C_BUS            1
#define OLED_I2C_BAUDRATE       400000

/* ==================== 宏定义 - WiFi SoftAP ==================== */

#define AP_SSID                 "Qian_Ru_Shi_Da_Sai"
#define AP_CHANNEL              6
#define AP_IP_OCT1              192
#define AP_IP_OCT2              168
#define AP_IP_OCT3              43
#define AP_IP_OCT4              1
#define UDP_MY_LISTEN_PORT      8082
#define UDP_CAR_TARGET_PORT     8081
#define UDP_BUFFER_SIZE         128

/* ==================== 宏定义 - 时序参数 ==================== */

#define BOOT_LED_BLINK_COUNT    5
#define BOOT_LED_BLINK_MS       250
#define BOOT_BEEP_MS            150
#define BEEP_ON_MS              200
#define BEEP_OFF_MS             200
#define KEY_DEBOUNCE_MS         30
#define KEY_CLICK_WINDOW_MS     500
#define KEY_HOLD_MS             300
#define OLED_REFRESH_MS         50
#define RSSI_BLINK_HALF_PERIOD_MS 500
#define WIFI_RSSI_INVALID       (-127)
#define HEARTBEAT_SEND_INTERVAL_MS 500
#define LINK_EVAL_INTERVAL_MS   1000
#define LINK_CONNECT_SUCCESS_COUNT 1
#define LINK_RECOVER_SUCCESS_COUNT 2
#define LINK_DISCONNECT_FAIL_COUNT 5

/* ==================== 宏定义 - OLED 布局 ==================== */

#define RSSI_BAR_HEIGHT          4
#define RSSI_BAR_BLOCKS          5

/* ==================== 宏定义 - 看门狗 ==================== */

#define WDT_TIMEOUT_SEC         10

/* ==================== 宏定义 - 收件箱 ==================== */

#define INBOX_MSG_LENGTH        32
#define INBOX_SHOW_DURATION_MS  2000

/* ==================== 类型定义 ==================== */

/* WiFi SoftAP 状态 */
typedef enum {
    WIFI_AP_STATE_INIT = 0,
    WIFI_AP_STATE_READY,
    WIFI_AP_STATE_ERROR,
} wifi_ap_state_t;

/* ==================== 全局变量 - 通信状态 ==================== */

static volatile uint8_t  g_wifi_ap_ok          = 0;   /* 1 = AP 启动成功 */
static volatile uint8_t  g_wifi_ap_state       = WIFI_AP_STATE_INIT;
static volatile uint8_t  g_wifi_link_ok        = 0;   /* 1 = 最近收到UDP状态 */
static volatile uint8_t  g_sle_link_ok         = 0;   /* 1 = SLE链路可用 */
static volatile uint8_t  g_sle_transport_ok    = 0;   /* 1 = SLE底层已连接 */
static volatile uint8_t  g_state_valid         = 0;   /* 1 = 最近收到有效状态 */
static volatile uint8_t  g_oled_ok             = 0;   /* OLED 初始化成功 */
static volatile uint8_t  g_lock_state          = 1;   /* 1=上锁 0=开锁 */
static volatile uint8_t  g_alarm_state         = 0;   /* 1=报警中 */
static volatile uint8_t  g_local_alarm         = 0;   /* 本地报警标志 */
static volatile uint8_t  g_alarm_buzzer_muted  = 0;   /* 1 = 移动端已静音 */
static volatile uint8_t  g_boot_done           = 0;   /* 上电自检完成 */
static volatile int32_t  g_wifi_rssi           = WIFI_RSSI_INVALID;
static volatile uint32_t g_wifi_window_elapsed_ms = 0;
static volatile uint32_t g_sle_window_elapsed_ms  = 0;
static volatile uint8_t  g_wifi_rx_in_window   = 0;
static volatile uint8_t  g_sle_rx_in_window    = 0;
static volatile uint8_t  g_wifi_success_streak = 0;
static volatile uint8_t  g_sle_success_streak  = 0;
static volatile uint8_t  g_wifi_fail_streak    = 0;
static volatile uint8_t  g_sle_fail_streak     = 0;
static volatile uint8_t  g_wifi_signal_blocks  = 0;

/* ==================== 全局变量 - 按键 ==================== */

static volatile uint8_t  g_button_pressed_flag = 0;   /* ISR 置位 */

/* ==================== 全局变量 - UDP ==================== */

static int               g_udp_socket          = -1;
static uint32_t          g_car_ip_address      = 0;    /* DHCP分配给车载端的IP */

/* ==================== 全局变量 - 收件箱 ==================== */

static char      g_inbox_latest_message[INBOX_MSG_LENGTH] = "EMPTY";
static volatile uint8_t g_inbox_has_message = 0;
static volatile uint8_t  g_inbox_show_flag = 0;
static volatile uint32_t g_inbox_show_elapsed_ms = 0;
static volatile uint8_t  g_wifi_bar_blink_visible = 1;

static void udp_send_command(const char *command);
static void inbox_add_message(const char *message);

static void mobile_trigger_inbox_display(void)
{
    if (g_inbox_show_flag) {
        return;
    }
    g_inbox_show_flag = 1;
    g_inbox_show_elapsed_ms = 0;
}

static void mobile_show_inbox_message(const char *message)
{
    inbox_add_message(message);
    g_inbox_show_flag = 1;
    g_inbox_show_elapsed_ms = 0;
}

/* ==================== 函数声明 - 收件箱 ==================== */

/**
 * @brief 向收件箱添加消息
 * @param message 消息字符串(最大31字节)
 */
static void inbox_add_message(const char *message)
{
    if (message == NULL) {
        return;
    }
    (void)strncpy_s(g_inbox_latest_message, INBOX_MSG_LENGTH,
                    message, INBOX_MSG_LENGTH - 1);
    g_inbox_latest_message[INBOX_MSG_LENGTH - 1] = '\0';
    g_inbox_has_message = 1;
}

static void mobile_update_local_alarm_output(void)
{
    if (!g_alarm_state) {
        g_alarm_buzzer_muted = 0;
        g_local_alarm = 0;
        return;
    }

    g_local_alarm = g_alarm_buzzer_muted ? 0 : 1;
}

static void mobile_refresh_wireless_flags(void)
{
    if (!g_wifi_link_ok && !g_sle_transport_ok && !g_sle_link_ok) {
        g_state_valid = 0;
        g_wifi_rssi = WIFI_RSSI_INVALID;
    }
}

static void mobile_record_link_rx(uint8_t from_sle)
{
    if (from_sle) {
        g_sle_rx_in_window = 1;
    } else {
        g_wifi_rx_in_window = 1;
    }
}

static void mobile_handle_wifi_link_sample(uint8_t success)
{
    if (success) {
        if (g_wifi_success_streak < 0xFF) {
            g_wifi_success_streak++;
        }
        g_wifi_fail_streak = 0;

        if (!g_wifi_link_ok) {
            if (g_wifi_success_streak >= LINK_CONNECT_SUCCESS_COUNT) {
                g_wifi_link_ok = 1;
                g_wifi_signal_blocks = RSSI_BAR_BLOCKS;
                mobile_show_inbox_message("WIFI CONNECTED");
                printf("[WIFI] link connected\r\n");
            }
            return;
        }

        if ((g_wifi_success_streak >= LINK_RECOVER_SUCCESS_COUNT) &&
            (g_wifi_signal_blocks < RSSI_BAR_BLOCKS)) {
            g_wifi_signal_blocks++;
            printf("[WIFI] signal recover=%u\r\n",
                   (unsigned int)g_wifi_signal_blocks);
        }
        return;
    }

    g_wifi_success_streak = 0;
    if (g_wifi_fail_streak < 0xFF) {
        g_wifi_fail_streak++;
    }

    if (!g_wifi_link_ok) {
        return;
    }

    if (g_wifi_signal_blocks > 0) {
        g_wifi_signal_blocks--;
    }
    printf("[WIFI] signal drop=%u fail=%u\r\n",
           (unsigned int)g_wifi_signal_blocks,
           (unsigned int)g_wifi_fail_streak);

    if (g_wifi_fail_streak >= LINK_DISCONNECT_FAIL_COUNT) {
        g_wifi_link_ok = 0;
        g_wifi_signal_blocks = 0;
        g_wifi_rssi = WIFI_RSSI_INVALID;
        mobile_show_inbox_message("WIFI DISCONNECTED");
        printf("[WIFI] link disconnected\r\n");
        mobile_refresh_wireless_flags();
    }
}

static void mobile_handle_sle_link_sample(uint8_t success)
{
    if (success) {
        if (g_sle_success_streak < 0xFF) {
            g_sle_success_streak++;
        }
        g_sle_fail_streak = 0;

        if (!g_sle_link_ok &&
            (g_sle_success_streak >= LINK_CONNECT_SUCCESS_COUNT)) {
            g_sle_link_ok = 1;
            mobile_show_inbox_message("SLE CONNECTED");
            printf("[SLE] link connected\r\n");
        }
        return;
    }

    g_sle_success_streak = 0;
    if (g_sle_fail_streak < 0xFF) {
        g_sle_fail_streak++;
    }

    if (!g_sle_link_ok) {
        return;
    }

    if (g_sle_fail_streak >= LINK_DISCONNECT_FAIL_COUNT) {
        g_sle_link_ok = 0;
        if (!g_sle_transport_ok) {
            mobile_show_inbox_message("SLE DISCONNECTED");
            printf("[SLE] link disconnected\r\n");
            mobile_refresh_wireless_flags();
        } else {
            printf("[SLE] data timeout but transport alive\r\n");
        }
    }
}

static void mobile_process_remote_message(const char *message, uint8_t from_sle)
{
    int lock_value;
    int alarm_value;
    int rssi_value = WIFI_RSSI_INVALID;

    if (message == NULL) {
        return;
    }

    mobile_record_link_rx(from_sle);

    if (sscanf(message, "STATE:%d,ALARM:%d,RSSI:%d",
               &lock_value, &alarm_value, &rssi_value) >= 2) {
        g_lock_state = (uint8_t)(lock_value ? 1 : 0);
        g_alarm_state = (uint8_t)(alarm_value ? 1 : 0);
        g_wifi_rssi = rssi_value;
        g_state_valid = 1;
        if (g_alarm_state) {
            g_alarm_buzzer_muted = 0;
        }
        mobile_update_local_alarm_output();
        return;
    }

    if (strcmp(message, "PING:WIFI") == 0) {
        return;
    }

    if (strcmp(message, "PING:SLE") == 0) {
        return;
    }

    if (strcmp(message, "ALARM:VIBRATION") == 0) {
        g_alarm_state = 1;
        g_alarm_buzzer_muted = 0;
        mobile_update_local_alarm_output();
        mobile_show_inbox_message("ALARM:VIBRATION");
        return;
    }

    if (strncmp(message, "INBOX:", 6) == 0) {
        const char *event_text = message + 6;

        if (strcmp(event_text, "VIBRATION ALERT") == 0) {
            g_alarm_state = 1;
            g_alarm_buzzer_muted = 0;
            mobile_update_local_alarm_output();
        }

        mobile_show_inbox_message(message + 6);
    }
}

static int mobile_get_wifi_signal_blocks(void)
{
    if (!g_wifi_link_ok) {
        return 0;
    }
    return (int)g_wifi_signal_blocks;
}

static void mobile_sle_notification_cb(uint8_t client_id,
                                       uint16_t conn_id,
                                       ssapc_handle_value_t *data,
                                       errcode_t status)
{
    UNUSED(client_id);
    UNUSED(conn_id);

    if ((status != ERRCODE_SUCC) || (data == NULL) || (data->data == NULL) ||
        (data->data_len == 0)) {
        return;
    }

    char buffer[UDP_BUFFER_SIZE];
    uint16_t copy_len = data->data_len;
    if (copy_len >= sizeof(buffer)) {
        copy_len = sizeof(buffer) - 1;
    }
    if (memcpy_s(buffer, sizeof(buffer), data->data, copy_len) != EOK) {
        return;
    }
    buffer[copy_len] = '\0';
    printf("[SLE] RX %s\r\n", buffer);
    mobile_process_remote_message(buffer, 1);
}

static void mobile_sle_indication_cb(uint8_t client_id,
                                     uint16_t conn_id,
                                     ssapc_handle_value_t *data,
                                     errcode_t status)
{
    mobile_sle_notification_cb(client_id, conn_id, data, status);
}

static errcode_t mobile_sle_send_command(const char *command)
{
    if (command == NULL) {
        return ERRCODE_INVALID_PARAM;
    }

    return sle_uart_client_send_data((const uint8_t *)command,
                                     (uint16_t)strlen(command));
}

static void mobile_link_monitor_update(uint32_t elapsed_ms)
{
    g_wifi_window_elapsed_ms += elapsed_ms;
    while (g_wifi_window_elapsed_ms >= LINK_EVAL_INTERVAL_MS) {
        g_wifi_window_elapsed_ms -= LINK_EVAL_INTERVAL_MS;
        mobile_handle_wifi_link_sample(g_wifi_rx_in_window);
        g_wifi_rx_in_window = 0;
    }

    g_sle_window_elapsed_ms += elapsed_ms;
    while (g_sle_window_elapsed_ms >= LINK_EVAL_INTERVAL_MS) {
        g_sle_window_elapsed_ms -= LINK_EVAL_INTERVAL_MS;
        mobile_handle_sle_link_sample((uint8_t)(g_sle_transport_ok ||
                                                g_sle_rx_in_window));
        g_sle_rx_in_window = 0;
    }
}

static void mobile_send_command(const char *command)
{
    if (command == NULL) {
        return;
    }

    if (mobile_sle_send_command(command) == ERRCODE_SUCC) {
        printf("[CTRL] SLE TX %s\r\n", command);
        return;
    }

    udp_send_command(command);
    printf("[CTRL] UDP TX %s\r\n", command);
}

/* ==================== 函数声明 - GPIO / ISR ==================== */

/**
 * @brief 按键下降沿中断服务函数
 * @note 直接触发收件箱显示 (g_inbox_show_flag = 1)
 */
static void button_isr_handler(pin_t pin, uintptr_t param)
{
    UNUSED(pin);
    UNUSED(param);

    /* ISR 中直接设置收件箱标志，display_task 会立刻刷新 */
    mobile_trigger_inbox_display();
    g_button_pressed_flag   = 1;
}

/**
 * @brief GPIO 初始化 (LED / 蜂鸣器 / 按键)
 * @note 必须在 OLED 初始化之后调用
 */
static void gpio_init_all(void)
{
    uapi_pin_init();
    uapi_gpio_init();

    /* LED: 输出, 复位后低电平 */
    uapi_pin_set_mode(PIN_LED, PIN_MODE_0);
    uapi_gpio_set_dir(PIN_LED, GPIO_DIRECTION_OUTPUT);
    uapi_gpio_set_val(PIN_LED, GPIO_LEVEL_LOW);

    /* 蜂鸣器: NPN有源蜂鸣器, 高电平响 */
    uapi_pin_set_mode(PIN_BUZZER, PIN_MODE_0);
    uapi_gpio_set_dir(PIN_BUZZER, GPIO_DIRECTION_OUTPUT);
    uapi_gpio_set_val(PIN_BUZZER, GPIO_LEVEL_LOW);

    /* 按键: 输入模式, 内部上拉, 下降沿触发 */
    uapi_pin_set_mode(PIN_KEY, PIN_MODE_0);
    uapi_gpio_set_dir(PIN_KEY, GPIO_DIRECTION_INPUT);
    uapi_pin_set_pull(PIN_KEY, PIN_PULL_TYPE_UP);
}

/* ==================== 函数声明 - 上电自检 ==================== */

/**
 * @brief 上电自检流程: LED闪烁5次 → 蜂鸣器150ms → LED灭
 */
static void boot_indicate(void)
{
    int i;
    for (i = 0; i < BOOT_LED_BLINK_COUNT; i++) {
        uapi_gpio_set_val(PIN_LED, GPIO_LEVEL_HIGH);
        osal_msleep(BOOT_LED_BLINK_MS);
        uapi_gpio_set_val(PIN_LED, GPIO_LEVEL_LOW);
        osal_msleep(BOOT_LED_BLINK_MS);
    }

    uapi_gpio_set_val(PIN_BUZZER, GPIO_LEVEL_HIGH);
    osal_msleep(BOOT_BEEP_MS);
    uapi_gpio_set_val(PIN_BUZZER, GPIO_LEVEL_LOW);

    uapi_gpio_set_val(PIN_LED, GPIO_LEVEL_LOW);
}

/* ==================== 函数声明 - OLED ==================== */

/**
 * @brief OLED 初始化 (I2C1, 400kHz)
 * @note 必须在 GPIO 初始化之前调用 (参照 helloworld_oled 示例)
 */
static void oled_init_hardware(void)
{
    uapi_pin_set_mode(PIN_OLED_SCL, PIN_MODE_2);
    uapi_pin_set_mode(PIN_OLED_SDA, PIN_MODE_2);

    if (uapi_i2c_master_init(OLED_I2C_BUS, OLED_I2C_BAUDRATE, 0) !=
        ERRCODE_SUCC) {
        printf("[OLED] 初始化失败, 检查I2C1引脚连接\r\n");
        return;
    }

    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();

    g_oled_ok = 1;
    printf("[OLED] 初始化成功\r\n");
}

/* ==================== 函数声明 - WiFi SoftAP ==================== */

/**
 * @brief WiFi SoftAP 初始化 (严格参照官方 sta_sample 调用顺序)
 * @return 0 成功, -1 失败
 */
static int wifi_softap_init(void)
{
    /* --- 官方方式: 延时5s等WiFi就绪, 不注册回调, 不调set_advance --- */
    printf("[AP] wait 5s...\r\n");
    osal_msleep(5000);
    printf("[AP] start\r\n");

    char ssid_buf[WIFI_MAX_SSID_LEN]  = AP_SSID;
    softap_config_stru cfg = {0};
    (void)memcpy_s(cfg.ssid, sizeof(cfg.ssid), ssid_buf, sizeof(ssid_buf));
    cfg.security_type = WIFI_SEC_TYPE_OPEN;
    cfg.channel_num   = AP_CHANNEL;

    if (wifi_softap_enable(&cfg) != 0) {
        printf("[AP] en fail\r\n");
        return -1;
    }
    printf("[AP] en ok\r\n");

    struct netif *nif = netif_find("ap0");
    if (!nif) { (void)wifi_softap_disable(); return -1; }

    ip4_addr_t ip, mk, gw;
    IP4_ADDR(&ip, AP_IP_OCT1, AP_IP_OCT2, AP_IP_OCT3, AP_IP_OCT4);
    IP4_ADDR(&mk, 255,255,255,0);
    IP4_ADDR(&gw, AP_IP_OCT1, AP_IP_OCT2, AP_IP_OCT3, AP_IP_OCT4);
    (void)netifapi_netif_set_addr(nif, &ip, &mk, &gw);
    (void)netifapi_dhcps_start(nif, NULL, 0);

    printf("[AP] OK 192.168.43.1\r\n");
    return 0;
}

/* ==================== 函数声明 - UDP 通信 ==================== */

/**
 * @brief 初始化 UDP Socket (绑定端口8082, SO_BROADCAST)
 */
static int mobile_udp_init(void)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        printf("[UDP] socket创建失败\r\n");
        return -1;
    }

    int broadcast_enable = 1;
    (void)setsockopt(sock, SOL_SOCKET, SO_BROADCAST,
                     (const void *)&broadcast_enable, sizeof(broadcast_enable));

    struct sockaddr_in bind_address = {0};
    bind_address.sin_family      = AF_INET;
    bind_address.sin_port        = htons(UDP_MY_LISTEN_PORT);
    bind_address.sin_addr.s_addr = INADDR_ANY;

    if (bind(sock, (struct sockaddr *)&bind_address,
             sizeof(bind_address)) < 0) {
        printf("[UDP] bind失败\r\n");
        lwip_close(sock);
        return -1;
    }

    printf("[UDP] 绑定端口 %d 成功\r\n", UDP_MY_LISTEN_PORT);
    return sock;
}

/**
 * @brief 通过 UDP 向车载端发送命令
 * @param command 命令字符串 ("UNLOCK" 或 "LOCK")
 */
static void udp_send_command(const char *command)
{
    if (g_udp_socket < 0 || g_car_ip_address == 0) {
        return;  /* UDP未初始化 或 尚未收到车载端IP */
    }

    struct sockaddr_in dest_address = {0};
    dest_address.sin_family      = AF_INET;
    dest_address.sin_port        = htons(UDP_CAR_TARGET_PORT);
    dest_address.sin_addr.s_addr = g_car_ip_address;

    (void)sendto(g_udp_socket, command, strlen(command), 0,
                 (struct sockaddr *)&dest_address, sizeof(dest_address));
}

/**
 * @brief 轮询接收车载端 UDP 状态广播
 * @note 首次收到有效包时记录车载端IP用于后续回发
 */
static void udp_poll_receive(void)
{
    if (g_udp_socket < 0) {
        return;
    }

    char receive_buffer[UDP_BUFFER_SIZE] = {0};
    struct sockaddr_in sender_address = {0};
    socklen_t sender_length = sizeof(sender_address);

    int received_bytes = recvfrom(g_udp_socket, receive_buffer,
                                   UDP_BUFFER_SIZE - 1,
                                   MSG_DONTWAIT,
                                   (struct sockaddr *)&sender_address,
                                   &sender_length);
    if (received_bytes <= 0) {
        return;
    }
    receive_buffer[received_bytes] = '\0';

    /* 首次收到车载端UDP包: 记录其IP地址 */
    if (g_car_ip_address == 0 && sender_address.sin_addr.s_addr != 0) {
        g_car_ip_address = sender_address.sin_addr.s_addr;
        printf("[UDP] 获取到车载端IP: %08X\r\n",
               (unsigned int)g_car_ip_address);
    }

    printf("[UDP] RX %s\r\n", receive_buffer);
    mobile_process_remote_message(receive_buffer, 0);
}

/* ==================== 函数声明 - 按键处理 ==================== */

/**
 * @brief 按键消费线程 (500ms窗口判断单击/双击/三击)
 * @note ISR 已设置收件箱显示标志, 此线程只处理三击和报警双击
 */
static void key_processing_thread(void *arg)
{
    UNUSED(arg);

    uint32_t click_window_start_time = 0;
    int      accumulated_clicks      = 0;

    printf("[按键] 任务启动, 等待按键事件\r\n");

    while (1) {
        if (g_button_pressed_flag) {
            g_button_pressed_flag = 0;

            osal_msleep(KEY_DEBOUNCE_MS);
            if (uapi_gpio_get_val(PIN_KEY) == GPIO_LEVEL_LOW) {
                uint32_t hold_start_time = osKernelGetTickCount();
                while (uapi_gpio_get_val(PIN_KEY) == GPIO_LEVEL_LOW) {
                    if ((osKernelGetTickCount() - hold_start_time) > KEY_HOLD_MS) {
                        break;
                    }
                    osal_msleep(5);
                }

                uint32_t release_time = osKernelGetTickCount();
                if (accumulated_clicks == 0) {
                    click_window_start_time = release_time;
                }
                accumulated_clicks++;
            }
        }

        uint32_t current_time = osKernelGetTickCount();
        if ((accumulated_clicks > 0) &&
            ((current_time - click_window_start_time) >= KEY_CLICK_WINDOW_MS)) {
            if (g_local_alarm || g_alarm_state) {
                if (accumulated_clicks == 2) {
                    g_alarm_buzzer_muted = 1;
                    g_local_alarm = 0;
                    printf("[KEY] local buzzer muted\r\n");
                }
            } else {
                if (accumulated_clicks == 3) {
                    const char *command = g_lock_state ? "UNLOCK" : "LOCK";
                    mobile_send_command(command);
                    printf("[按键] 发送命令: %s\r\n", command);
                }
            }
            accumulated_clicks = 0;
        }

        osal_msleep(10);
    }
}

/* ==================== 函数声明 - OLED 显示 ==================== */

/**
 * @brief 绘制底部 RSSI 信号强度条
 */
static void oled_draw_rssi_bar(void)
{
    int active_blocks;
    int bar_pixel_width = 108 / RSSI_BAR_BLOCKS;
    uint8_t bar_y_position = SSD1306_HEIGHT - RSSI_BAR_HEIGHT - 2;
    int i;

    if (g_wifi_link_ok) {
        active_blocks = mobile_get_wifi_signal_blocks();
    } else if (g_wifi_bar_blink_visible) {
        active_blocks = -1;
    } else {
        return;
    }

    for (i = 0; i < RSSI_BAR_BLOCKS; i++) {
        uint8_t x_start = (uint8_t)(8 + i * bar_pixel_width);
        uint8_t x_end = (uint8_t)(x_start + bar_pixel_width - 3);
        uint8_t fill_y;

        ssd1306_DrawRectangle(x_start, bar_y_position, x_end,
                              (uint8_t)(bar_y_position + RSSI_BAR_HEIGHT),
                              White);

        if ((active_blocks < 0) || (i >= active_blocks)) {
            continue;
        }

        for (fill_y = (uint8_t)(bar_y_position + 1);
             fill_y < (uint8_t)(bar_y_position + RSSI_BAR_HEIGHT);
             fill_y++) {
            ssd1306_DrawLine((uint8_t)(x_start + 1), fill_y,
                             (uint8_t)(x_end - 1), fill_y, White);
        }
    }
}

/**
 * @brief 收件箱界面 (覆盖全屏, 显示最近4条消息)
 */
static void oled_display_inbox(void)
{
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_DrawString("INBOX", Font_7x10, White);
    ssd1306_SetCursor(0, 18);
    ssd1306_DrawString(g_inbox_has_message ? g_inbox_latest_message : "EMPTY",
                       Font_7x10, White);
    ssd1306_UpdateScreen();
}

/**
 * @brief 正常显示界面 (4行数据 + RSSI信号条)
 */
static void oled_display_normal(void)
{
    uint8_t state_visible;

    mobile_refresh_wireless_flags();
    state_visible = (uint8_t)(g_state_valid &&
                              (g_wifi_link_ok || g_sle_link_ok));
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    if (state_visible) {
        ssd1306_DrawString(g_lock_state ? "LOCK:LOCKED" : "LOCK:UNLOCKED",
                           Font_7x10, White);
    } else {
        ssd1306_DrawString("LOCK:UNKNOWN", Font_7x10, White);
    }

    ssd1306_SetCursor(0, 14);
    if (state_visible) {
        ssd1306_DrawString(g_alarm_state ?
                           "VIBRATION:YES" : "VIBRATION:NO",
                           Font_7x10, White);
    } else {
        ssd1306_DrawString("VIBRATION:UNKNOWN", Font_7x10, White);
    }

    ssd1306_SetCursor(0, 28);
    ssd1306_DrawString(g_sle_link_ok ? "SLE:OK" : "SLE:NO",
                       Font_7x10, White);

    ssd1306_SetCursor(0, 42);
    ssd1306_DrawString(g_wifi_link_ok ? "WIFI:OK" : "WIFI:NO",
                       Font_7x10, White);

    oled_draw_rssi_bar();

    ssd1306_UpdateScreen();
}

/**
 * @brief OLED 显示刷新线程 (每500ms刷新)
 * @note 收件箱模式优先, 持续2秒后自动恢复
 */
static void oled_display_thread(void *arg)
{
    UNUSED(arg);
    uint32_t blink_elapsed_ms = 0;

    while (1) {
        if (!g_oled_ok) {
            osal_msleep(OLED_REFRESH_MS);
            continue;
        }

        blink_elapsed_ms += OLED_REFRESH_MS;
        if (blink_elapsed_ms >= RSSI_BLINK_HALF_PERIOD_MS) {
            blink_elapsed_ms = 0;
            g_wifi_bar_blink_visible = g_wifi_bar_blink_visible ? 0 : 1;
        }

        /* 收件箱模式: 检查是否超时 */
        if (g_inbox_show_flag) {
            oled_display_inbox();
            osal_msleep(OLED_REFRESH_MS);
            g_inbox_show_elapsed_ms += OLED_REFRESH_MS;
            if (g_inbox_show_elapsed_ms >= INBOX_SHOW_DURATION_MS) {
                g_inbox_show_flag = 0;
                g_inbox_show_elapsed_ms = 0;
            }
            continue;
        }

        /* 正常显示 */
        oled_display_normal();
        osal_msleep(OLED_REFRESH_MS);
    }
}

/* ==================== 函数声明 - 蜂鸣器 ==================== */

/**
 * @brief 蜂鸣器线程 (报警时 200ms ON / 200ms OFF 交替)
 */
static void buzzer_thread(void *arg)
{
    UNUSED(arg);

    /* 等待自检完成 */
    while (!g_boot_done) {
        osal_msleep(50);
    }

    while (1) {
        if (g_local_alarm) {
            uapi_gpio_set_val(PIN_BUZZER, GPIO_LEVEL_HIGH);
            osal_msleep(BEEP_ON_MS);
            uapi_gpio_set_val(PIN_BUZZER, GPIO_LEVEL_LOW);
            osal_msleep(BEEP_OFF_MS);
        } else {
            uapi_gpio_set_val(PIN_BUZZER, GPIO_LEVEL_LOW);
            osal_msleep(100);
        }
    }
}

static void mobile_sle_thread(void *arg)
{
    UNUSED(arg);
    uint8_t last_sle_connected = 0;

    while (!g_boot_done) {
        osal_msleep(100);
    }

    printf("[SLE] CLIENT INIT\r\n");
    sle_uart_client_init(mobile_sle_notification_cb, mobile_sle_indication_cb);

    while (1) {
        uint8_t sle_connected = sle_uart_client_is_connected() ? 1 : 0;
        if (sle_connected != last_sle_connected) {
            g_sle_transport_ok = sle_connected;
            printf("[SLE] transport %s\r\n",
                   sle_connected ? "connected" : "disconnected");
            if (sle_connected) {
                if (!g_sle_link_ok) {
                    g_sle_link_ok = 1;
                    mobile_show_inbox_message("SLE CONNECTED");
                }
                g_sle_fail_streak = 0;
                g_sle_success_streak = LINK_CONNECT_SUCCESS_COUNT;
            } else {
                g_sle_rx_in_window = 0;
                g_sle_success_streak = 0;
                g_sle_link_ok = 0;
                mobile_show_inbox_message("SLE DISCONNECTED");
            }
            last_sle_connected = sle_connected;
        }
        mobile_refresh_wireless_flags();
        osal_msleep(200);
    }
}

static void mobile_heartbeat_thread(void *arg)
{
    UNUSED(arg);

    while (!g_boot_done) {
        osal_msleep(100);
    }

    while (1) {
        if ((g_udp_socket >= 0) && (g_car_ip_address != 0)) {
            udp_send_command("PING:WIFI");
        }
        if (sle_uart_client_is_ready()) {
            (void)mobile_sle_send_command("PING:SLE");
        }
        osal_msleep(HEARTBEAT_SEND_INTERVAL_MS);
    }
}

/* ==================== 主控线程 ==================== */

/**
 * @brief 主控线程: 初始化所有硬件 → 启动WiFi AP → UDP监听循环
 */
static void mobile_main_thread(void *arg)
{
    UNUSED(arg);

    /* --- 第1步: OLED 初始化 (必须在 GPIO init 之前) --- */
    oled_init_hardware();

    /* --- 第2步: GPIO 初始化 (LED/蜂鸣器/按键) --- */
    gpio_init_all();

    /* --- 第3步: 注册按键中断 --- */
    uapi_gpio_register_isr_func(PIN_KEY, GPIO_INTERRUPT_FALLING_EDGE,
                                 button_isr_handler);
    printf("[主控] GPIO和ISR注册完成\r\n");

    /* --- 第4步: 上电自检 --- */
    boot_indicate();
    g_boot_done = 1;

    /* --- 第5步: 启动 WiFi SoftAP --- */
    if (wifi_softap_init() == 0) {
        g_wifi_ap_ok    = 1;
        g_wifi_ap_state = WIFI_AP_STATE_READY;
        uapi_gpio_set_val(PIN_LED, GPIO_LEVEL_HIGH);
        printf("[主控] AP启动成功, LED已点亮\r\n");
    } else {
        printf("[主控] AP启动失败\r\n");
    }

    /* --- 第6步: UDP 初始化 --- */
    g_udp_socket = mobile_udp_init();

    /* --- 第7步: 看门狗 --- */
    uapi_watchdog_init(WDT_TIMEOUT_SEC);
    uapi_watchdog_enable(WDT_MODE_RESET);

    /* --- 主循环: UDP 接收 + 喂狗 --- */
    while (1) {
        uapi_watchdog_kick();
        udp_poll_receive();
        mobile_link_monitor_update(50);
        osal_msleep(50);
    }
}

/* ==================== 系统入口 ==================== */

/**
 * @brief 应用入口: 创建4个线程
 *   - MobileMain:  主控 (WiFi AP + UDP + WDT)
 *   - MobileDisp:  OLED 显示刷新
 *   - MobileKey:   按键消费
 *   - MobileBuzz:  蜂鸣器
 */
static void mobile_application_entry(void)
{
    osThreadAttr_t thread_attrs = {0};

    /* 主控线程 */
    thread_attrs.name       = "MobileMain";
    thread_attrs.stack_size = 0x4000;
    thread_attrs.priority   = osPriorityNormal;
    (void)osThreadNew((osThreadFunc_t)mobile_main_thread, NULL, &thread_attrs);

    /* OLED 显示线程 */
    thread_attrs.name       = "MobileDisp";
    thread_attrs.stack_size = 0x2000;
    thread_attrs.priority   = osPriorityBelowNormal;
    (void)osThreadNew((osThreadFunc_t)oled_display_thread, NULL,
                      &thread_attrs);

    /* 按键处理线程 */
    thread_attrs.name       = "MobileKey";
    thread_attrs.stack_size = 0x1000;
    thread_attrs.priority   = osPriorityNormal;
    (void)osThreadNew((osThreadFunc_t)key_processing_thread, NULL,
                      &thread_attrs);

    /* 蜂鸣器线程 */
    thread_attrs.name       = "MobileBuzz";
    thread_attrs.stack_size = 0x1000;
    thread_attrs.priority   = osPriorityNormal;
    (void)osThreadNew((osThreadFunc_t)buzzer_thread, NULL, &thread_attrs);

    /* SLE 客户端线程 */
    thread_attrs.name       = "MobileSLE";
    thread_attrs.stack_size = 0x2000;
    thread_attrs.priority   = osPriorityBelowNormal;
    (void)osThreadNew((osThreadFunc_t)mobile_sle_thread, NULL,
                      &thread_attrs);

    thread_attrs.name       = "MobileHB";
    thread_attrs.stack_size = 0x1000;
    thread_attrs.priority   = osPriorityBelowNormal;
    (void)osThreadNew((osThreadFunc_t)mobile_heartbeat_thread, NULL,
                      &thread_attrs);
}

app_run(mobile_application_entry);
