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

#include "ble_remote.h"

/* Adapted from Dr-Vah/Project-of-Electronic-Circuits dev 2b072bb, BLEControl. */

#define TAG "BLE_CAR"

// Nordic UART UUIDs, least-significant byte first (corrected from upstream).
#define SERVICE_UUID_128 { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
                           0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E }
#define RX_CHAR_UUID_128  { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
                           0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E }
#define STATUS_UUID_128 { 0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, \
                         0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E }
#define DEVICE_NAME "Omni-Remote-BLE"
#define PROFILE_APP_ID 0

// Maximum ATT value length. Parsing and speed limits live in ble_protocol.h.
#define FRAME_MAX_LEN 32

enum {
    IDX_SVC,
    IDX_RX_CHAR,
    IDX_RX_VAL,
    IDX_STATUS_CHAR,
    IDX_STATUS_VAL,
    IDX_STATUS_CCC,
    IDX_NB,
};

static uint8_t service_uuid_128[16] = SERVICE_UUID_128;
static uint8_t rx_char_uuid_128[16] = RX_CHAR_UUID_128;
static uint8_t status_uuid_128[16] = STATUS_UUID_128;
static const uint8_t char_prop_read = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint16_t ccc_uuid=ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static uint8_t ccc_value[2];
static uint8_t status_value[BLE_STATUS_SIZE];

static const uint8_t primary_service_uuid[2] = {0x00, 0x28};  // 0x2800
static const uint8_t char_decl_uuid[2] = {0x03, 0x28};        // 0x2803
static const uint8_t char_prop_write =
    ESP_GATT_CHAR_PROP_BIT_WRITE;
static uint8_t rx_value[FRAME_MAX_LEN] = {0};

static esp_gatts_attr_db_t gatt_db[IDX_NB] = {
    // 服务声明：值是 128-bit 服务 UUID
    [IDX_SVC] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
         sizeof(service_uuid_128), sizeof(service_uuid_128), service_uuid_128}
    },
    // Acknowledged writes let the client detect rejection of its driving lease.
    [IDX_RX_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)&char_decl_uuid, ESP_GATT_PERM_READ,
         sizeof(char_prop_write), sizeof(char_prop_write),
         (uint8_t *)&char_prop_write}
    },
    // 写特征值本身（可写）
    [IDX_RX_VAL] = {
        {ESP_GATT_RSP_BY_APP},
        {ESP_UUID_LEN_128, rx_char_uuid_128, ESP_GATT_PERM_WRITE,
         sizeof(rx_value), 0, rx_value}
    },
    [IDX_STATUS_CHAR] = {
        {ESP_GATT_AUTO_RSP},
        {ESP_UUID_LEN_16, (uint8_t *)char_decl_uuid, ESP_GATT_PERM_READ,
         1, 1, (uint8_t *)&char_prop_read}
    },
    [IDX_STATUS_VAL] = {
        {ESP_GATT_RSP_BY_APP},
        {ESP_UUID_LEN_128, status_uuid_128, ESP_GATT_PERM_READ,
         sizeof(status_value), sizeof(status_value), status_value}
    },
    [IDX_STATUS_CCC] = {
        {ESP_GATT_RSP_BY_APP},
        {ESP_UUID_LEN_16,(uint8_t *)&ccc_uuid,ESP_GATT_PERM_READ|ESP_GATT_PERM_WRITE,
         2,2,ccc_value}
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
static bool s_connected;
static bool s_subscribed;
static esp_gatt_if_t s_gatts_if=ESP_GATT_IF_NONE;
static portMUX_TYPE link_mux=portMUX_INITIALIZER_UNLOCKED;
static ble_remote_command_fn on_command;
static ble_remote_link_fn on_link;
static ble_remote_status_fn on_status;

static void status_task(void *arg) {
    while(true) {
        vTaskDelay(pdMS_TO_TICKS(500));
        portENTER_CRITICAL(&link_mux);
        bool notify=s_connected&&s_subscribed;
        uint16_t conn=s_conn_id,handle=s_handle_table[IDX_STATUS_VAL];
        esp_gatt_if_t interface=s_gatts_if;
        portEXIT_CRITICAL(&link_mux);
        if(notify) {
            uint8_t value[BLE_STATUS_SIZE];on_status(value);
            /* A dropped notification never renews or revokes driving ownership. */
            esp_ble_gatts_send_indicate(interface,conn,handle,sizeof(value),value,false);
        }
    }
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
        portENTER_CRITICAL(&link_mux);
        s_conn_id = param->connect.conn_id;
        s_connected = true;
        s_subscribed=false;s_gatts_if=gatts_if;
        portEXIT_CRITICAL(&link_mux);
        on_link(true);
        ESP_LOGI(TAG, "已连接 conn_id=%d", s_conn_id);
        break;

    case ESP_GATTS_DISCONNECT_EVT:
        portENTER_CRITICAL(&link_mux);
        s_connected = false;
        s_subscribed=false;
        s_conn_id = 0;
        portEXIT_CRITICAL(&link_mux);
        on_link(false);
        ESP_LOGI(TAG, "已断开，重新开始广播...");
        esp_ble_gap_start_advertising(&adv_params);
        break;

    case ESP_GATTS_READ_EVT: {
        esp_gatt_rsp_t response={0};
        esp_gatt_status_t status=ESP_GATT_READ_NOT_PERMIT;
        if(param->read.handle==s_handle_table[IDX_STATUS_VAL]) {
            if(param->read.offset!=0) status=ESP_GATT_INVALID_OFFSET;
            else {
                status=ESP_GATT_OK;
                response.attr_value.handle=param->read.handle;
                response.attr_value.len=BLE_STATUS_SIZE;
                on_status(response.attr_value.value);
            }
        } else if(param->read.handle==s_handle_table[IDX_STATUS_CCC]&&param->read.offset==0) {
            status=ESP_GATT_OK;response.attr_value.handle=param->read.handle;
            response.attr_value.len=2;response.attr_value.value[0]=s_subscribed?1:0;
        }
        esp_ble_gatts_send_response(gatts_if,param->read.conn_id,
            param->read.trans_id,status,&response);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        bool ok = false;
        if(param->write.handle==s_handle_table[IDX_STATUS_CCC] &&
           !param->write.is_prep && param->write.offset==0 && param->write.len==2 &&
           param->write.value[1]==0 && param->write.value[0]<=1) {
            portENTER_CRITICAL(&link_mux);
            s_subscribed=param->write.value[0]==1;
            portEXIT_CRITICAL(&link_mux);ok=true;
        }
        if (s_connected && param->write.conn_id == s_conn_id &&
            param->write.handle == s_rx_handle && !param->write.is_prep &&
            param->write.offset == 0) {
            ble_command_t command = ble_parse(param->write.value, param->write.len);
            if (command.kind != BLE_INVALID) ok = on_command(command);
        }
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                param->write.trans_id, ok ? ESP_GATT_OK : ESP_GATT_ERROR, NULL);
        }
        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        esp_ble_gatts_send_response(gatts_if, param->exec_write.conn_id,
            param->exec_write.trans_id, ESP_GATT_REQ_NOT_SUPPORTED, NULL);
        break;

    default:
        break;
    }
}

/* NVS is initialized by app_main. Motor access stays in its shared control task. */
esp_err_t ble_remote_start(ble_remote_command_fn command, ble_remote_link_fn link,
                           ble_remote_status_fn status)
{
    on_command = command; on_link = link; on_status = status;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_err_t e;
#define BLE_TRY(call) do { e=(call); if(e!=ESP_OK) return e; } while(0)
    BLE_TRY(esp_bt_controller_init(&bt_cfg));
    BLE_TRY(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    BLE_TRY(esp_bluedroid_init());
    BLE_TRY(esp_bluedroid_enable());
    BLE_TRY(esp_ble_gap_set_device_name(DEVICE_NAME));
    BLE_TRY(esp_ble_gap_register_callback(gap_event_handler));
    BLE_TRY(esp_ble_gatts_register_callback(gatts_event_handler));
    BLE_TRY(esp_ble_gatts_app_register(PROFILE_APP_ID));
    if(xTaskCreate(status_task,"ble_status",3072,NULL,2,NULL)!=pdPASS)return ESP_ERR_NO_MEM;
#undef BLE_TRY
    ESP_LOGI(TAG, "BLE controller: %s", DEVICE_NAME);
    return ESP_OK;
}
