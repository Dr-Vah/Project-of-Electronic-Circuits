#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "car_control.h"

#define TAG "BLE_CAR"

// ============================================================================
// 通信协议（与微信小程序保持一致）
//   广播名：ESP32_CAR
//   服务 UUID：      6E400001-B5A3-F393-E0A9-E50E24DCCA9E (Nordic UART Service)
//   写特征值 UUID：  6E400002-B5A3-F393-E0A9-E50E24DCCA9E (可写 / Write)
//   数据帧：         "#vx,vy,w!"  vx=左右(X)、vy=前后(Y) (m/s)，w=旋转角速度 (rad/s)
// ============================================================================

// 128-bit UUID 在 BLE 空口上按小端传输，字节顺序与字符串相反。
#define SERVICE_UUID_128 { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
                           0x93, 0xF3, 0xA3, 0xB5, 0x6E, 0x40, 0x00, 0x01 }
#define RX_CHAR_UUID_128  { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
                           0x93, 0xF3, 0xA3, 0xB5, 0x6E, 0x40, 0x00, 0x02 }

#define DEVICE_NAME "ESP32_CAR"
#define PROFILE_APP_ID 0

// 数据帧最大长度、限幅速度、超时停车时间
#define FRAME_MAX_LEN 32
#define MAX_SPEED_MPS 0.5f
#define MAX_OMEGA_RAD_S 1.5f
#define COMMAND_TIMEOUT_US (500 * 1000)

enum {
    IDX_SVC,
    IDX_RX_CHAR,
    IDX_RX_VAL,
    IDX_NB,
};

static uint8_t service_uuid_128[16] = SERVICE_UUID_128;
static uint8_t rx_char_uuid_128[16] = RX_CHAR_UUID_128;

static const uint8_t primary_service_uuid[2] = {0x00, 0x28};  // 0x2800
static const uint8_t char_decl_uuid[2] = {0x03, 0x28};        // 0x2803
static const uint8_t char_prop_write =
    ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_WRITE;
static uint8_t rx_value[FRAME_MAX_LEN] = {0};

static esp_gatts_attr_db_t gatt_db[IDX_NB] = {
    // 服务声明：值是 128-bit 服务 UUID
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
         sizeof(service_uuid_128), sizeof(service_uuid_128), service_uuid_128}
    },
    // 写特征值声明（属性字节：Write / Write No Response）
    [IDX_RX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid, ESP_GATT_PERM_READ,
         sizeof(char_prop_write), sizeof(char_prop_write),
         (uint8_t *)&char_prop_write}
    },
    // 写特征值本身（可写）
    [IDX_RX_VAL] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_128, rx_char_uuid_128, ESP_GATT_PERM_WRITE,
         sizeof(rx_value), 0, rx_value}
    },
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static uint16_t s_handle_table[IDX_NB] = {0};
static uint16_t s_service_handle = 0;
static uint16_t s_rx_handle = 0;
static uint16_t s_conn_id = 0;
static volatile bool s_connected = false;
static volatile int64_t s_last_command_us = 0;

static float clamp_speed(float value)
{
    if (!isfinite(value)) {
        return 0.0f;
    }
    if (value < -MAX_SPEED_MPS) {
        return -MAX_SPEED_MPS;
    }
    if (value > MAX_SPEED_MPS) {
        return MAX_SPEED_MPS;
    }
    // 死区：接近 0 直接归零，避免电机微小抖动
    if (fabsf(value) < 0.01f) {
        return 0.0f;
    }
    return value;
}

static float clamp_omega(float value)
{
    if (!isfinite(value)) {
        return 0.0f;
    }
    if (value < -MAX_OMEGA_RAD_S) {
        return -MAX_OMEGA_RAD_S;
    }
    if (value > MAX_OMEGA_RAD_S) {
        return MAX_OMEGA_RAD_S;
    }
    // 死区：接近 0 直接归零，避免自转抖动
    if (fabsf(value) < 0.02f) {
        return 0.0f;
    }
    return value;
}

static void apply_command(float vx, float vy, float w)
{
    vx = clamp_speed(vx);
    vy = clamp_speed(vy);
    w = clamp_omega(w);

    // 手机坐标系 → 底盘坐标系：
    //   手机右倾 vx>0 → 底盘 +x（右），vx 不取反；
    //   手机前倾 vy<0 → 底盘 +y（前进），故 vy 取反；
    //   手机逆时针转（陀螺仪 z>0）→ 底盘 omega>0（逆时针），w 直接用；方向反了改符号。
    car_control_set_velocity(vx, vy, w);

    s_last_command_us = esp_timer_get_time();
    ESP_LOGI(TAG, "drive vx=%.2f vy=%.2f w=%.2f", vx, vy, w);
}

static void parse_and_drive(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0 || len >= FRAME_MAX_LEN) {
        return;
    }

    char buf[FRAME_MAX_LEN];
    memcpy(buf, data, len);
    buf[len] = '\0';

    if (buf[0] != '#') {
        return;
    }

    char *comma = strchr(buf, ',');
    char *bang = strchr(buf, '!');
    if (comma == NULL || bang == NULL || comma >= bang) {
        return;
    }

    // 第二个逗号（w 值之前）
    char *comma2 = strchr(comma + 1, ',');
    if (comma2 == NULL || comma2 >= bang) {
        return;
    }

    *comma = '\0';
    *comma2 = '\0';
    *bang = '\0';

    const float vx = (float)atof(buf + 1);
    const float vy = (float)atof(comma + 1);
    const float w = (float)atof(comma2 + 1);
    apply_command(vx, vy, w);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event,
                              esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: {
        // 广播包配置完成，接着配置扫描响应（携带完整设备名）
        esp_ble_adv_data_t scan_rsp = {
            .set_scan_rsp = true,
            .include_name = true,
            .include_txpower = true,
        };
        esp_ble_gap_config_adv_data(&scan_rsp);
        break;
    }
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&adv_params);
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
    case ESP_GATTS_REG_EVT:
        ESP_ERROR_CHECK(esp_ble_gatts_create_attr_tab(
            gatt_db, gatts_if, IDX_NB, PROFILE_APP_ID));
        break;

    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "创建属性表失败 0x%x", param->add_attr_tab.status);
            break;
        }
        if (param->add_attr_tab.num_handle != IDX_NB) {
            ESP_LOGE(TAG, "属性表句柄数量异常 %d != %d",
                     param->add_attr_tab.num_handle, IDX_NB);
            break;
        }
        memcpy(s_handle_table, param->add_attr_tab.handles,
               sizeof(s_handle_table));
        s_service_handle = s_handle_table[IDX_SVC];
        s_rx_handle = s_handle_table[IDX_RX_VAL];
        esp_ble_gatts_start_service(s_service_handle);

        ESP_LOGI(TAG, "GATT 服务已启动，开始广播...");
        esp_ble_gap_config_adv_data(&(esp_ble_adv_data_t){
            .set_scan_rsp = false,
            .include_name = false,
            .include_txpower = false,
            .service_uuid_len = sizeof(service_uuid_128),
            .p_service_uuid = service_uuid_128,
            .flag = (ESP_BLE_ADV_FLAG_GEN_DISC |
                     ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
        });
        break;

    case ESP_GATTS_CONNECT_EVT:
        s_conn_id = param->connect.conn_id;
        s_connected = true;
        s_last_command_us = esp_timer_get_time();
        ESP_LOGI(TAG, "已连接 conn_id=%d", s_conn_id);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        s_connected = false;
        s_conn_id = 0;
        car_control_stop();
        ESP_LOGI(TAG, "已断开，重新开始广播...");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_WRITE_EVT:
        if (param->write.handle == s_rx_handle) {
            parse_and_drive(param->write.value, param->write.len);
        }
        break;

    default:
        break;
    }
}

// 超时停车看门狗：连接状态下超过 COMMAND_TIMEOUT_US 没收到指令就停车
static void command_watchdog_task(void *arg)
{
    (void)arg;
    while (true) {
        if (s_connected &&
            (esp_timer_get_time() - s_last_command_us) > COMMAND_TIMEOUT_US) {
            car_control_stop();
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    const car_control_config_t car_config = CAR_CONTROL_DEFAULT_CONFIG();
    ESP_ERROR_CHECK(car_control_init(&car_config));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(DEVICE_NAME));

    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(PROFILE_APP_ID));

    xTaskCreate(command_watchdog_task, "cmd_watchdog", 2048, NULL, 5, NULL);

    ESP_LOGI(TAG, "BLE 体感遥控已启动，等待连接...");
}
