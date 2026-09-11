#pragma once
#include <stdbool.h>
#include "esp_err.h"
#include "ble_protocol.h"
typedef bool (*ble_remote_command_fn)(ble_command_t command);
typedef void (*ble_remote_link_fn)(bool connected);
/* v1, dizzy, fatigue, idle seconds, face, manual+1, fault, BLE owns lease. */
#define BLE_STATUS_SIZE 8
typedef void (*ble_remote_status_fn)(uint8_t status[BLE_STATUS_SIZE]);
esp_err_t ble_remote_start(ble_remote_command_fn command, ble_remote_link_fn link,
                           ble_remote_status_fn status);
