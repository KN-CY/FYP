#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <time.h>

#include "nvs_flash.h"
#include "esp_partition.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "uECC.h"

#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_sntp.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#define CHECK_BIT(var,pos) ((var) & (1<<(7-pos)))

#define TEST_RTS (18)
#define TEST_CTS (18)

#define UART_PORT_NUM      (0)
#define UART_BAUD_RATE     (115200)
#define TASK_STACK_SIZE    (2048)

#define BUF_SIZE (1024)

// Advertsising related
#define INTERVAL_TIME 0x0020 // units of 0.625ms. 0x20 * 0.625ms = 20ms, Apple's recommendation

// Scan related
#define MAX_UNIQUE_MACS 100
#define MIN_PACKET_COUNT 0
#define SCAN_TIME 1000 // in ms
static int apple_packet_count = 0;

static esp_bd_addr_t unique_macs_09[MAX_UNIQUE_MACS];
static esp_bd_addr_t unique_macs_10[MAX_UNIQUE_MACS];
static esp_bd_addr_t unique_macs_16[MAX_UNIQUE_MACS];

static int unique_macs_09_packet_count[MAX_UNIQUE_MACS] = {0};
static int unique_macs_10_packet_count[MAX_UNIQUE_MACS] = {0};
static int unique_macs_16_packet_count[MAX_UNIQUE_MACS] = {0};

static int unique_macs_09_count = 0;
static int unique_macs_10_count = 0;
static int unique_macs_16_count = 0;

static const char* LOG_TAG = "findmy_modem";
static const char* MY_LOG = "My debug:";

static void print_manufacturer_data(esp_bd_addr_t bd_addr, int manufacturer_data_length, uint8_t* manufacturer_data, int flag_data_length, uint8_t* flag_data) {
    printf("MAC Address: ");
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
        printf("%02X:", bd_addr[i]);
    }
    printf(" | ");
    printf("Manufacturer data: ");
    for (int i = 0; i < manufacturer_data_length; i++) {
        printf("%02X ", manufacturer_data[i]);
    }

    printf(" | ");
    if (flag_data_length > 0) { 
        printf("Flag data: ");
        for (int i = 0; i < flag_data_length; i++) {
            printf("%02X ", flag_data[i]);
        }
    } else {
        printf("No Flag");
    }

    printf("\n");
}

static void print_unique_mac() {
    printf("There are %d unique MAC for 0x09: \n", unique_macs_09_count);
    for (int i = 0; i < unique_macs_09_count; i++) {
        for (int j = 0; j < ESP_BD_ADDR_LEN; j++) {
            printf("%02X:", unique_macs_09[i][j]);
        }
        printf("\n");
    }

    printf("There are %d unique MAC for 0x10: \n", unique_macs_10_count);
    for (int i = 0; i < unique_macs_10_count; i++) {
        for (int j = 0; j < ESP_BD_ADDR_LEN; j++) {
            printf("%02X:", unique_macs_10[i][j]);
        }
        printf("\n");
    }
   
    printf("There are %d unique MAC for 0x16: \n", unique_macs_16_count);
    for (int i = 0; i < unique_macs_16_count; i++) {
        for (int j = 0; j < ESP_BD_ADDR_LEN; j++) {
            printf("%02X:", unique_macs_16[i][j]);
        }
        printf("\n");
    }

}



/** Callback function for BT events */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

/** Random device address */
static esp_bd_addr_t rnd_addr = { 0xFF, 0xBB, 0xCB, 0xDD, 0xEE, 0xFF };

/** Advertisement payload */
static uint8_t adv_data[31] = {
    0x1e, /* Length (30) */
    0xff, /* Manufacturer Specific Data (type 0xff) */
    0x4c, 0x00, /* Company ID (Apple) */
    0x12, 0x19, /* Offline Finding type and length */
    0x00, /* State */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, /* First two bits */
    0x00, /* Hint (0x00) */
};

uint8_t start_addr[16] = {
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00, 
    0x00, 0x00, 0x00, 0x00
};

uint8_t curr_addr[16];  



static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE, // originally  BLE_SCAN_TYPE_ACTIVE
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x30, // 0x40 == 0.04 // 0x640 == 1s scan interval // originally 0x50
    .scan_window            = 0x30 // 0x30 == 0.030s // 0x20 == 0.020s scan interval // originally 0x30
};// param definitions: https://github.com/pycom/esp-idf-2.0/blob/master/components/bt/bluedroid/api/include/esp_gap_ble_api.h#L203

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    switch (event) {
        // Scan related
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            // printf("K: ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT called\n");
            if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                printf("Scan parameters set, start scanning for %d ms...\n", SCAN_TIME);
                esp_ble_gap_start_scanning(SCAN_TIME/1000); // units in second 
            } else {
                printf("Unable to set scan parameters, error code %d\n", param->scan_param_cmpl.status);
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT: {
            printf("K: ESP_GAP_BLE_SCAN_START_COMPLETE_EVT called\n");
            if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                printf("Scan start failed, error code %d\n", param->scan_start_cmpl.status);
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            // printf("K: ESP_GAP_BLE_SCAN_RESULT_EVT called\n");
            esp_ble_gap_cb_param_t *scan_result = param;

            esp_bd_addr_t bd_addr; // K: Stores the mac address of the BLE device
            memcpy(bd_addr, scan_result->scan_rst.bda, sizeof(bd_addr));

            esp_bt_dev_type_t dev_type = scan_result->scan_rst.dev_type ; // Flag for device capability

            esp_ble_addr_type_t add_type = scan_result->scan_rst.ble_addr_type;

            esp_ble_evt_type_t event_type = scan_result->scan_rst.ble_evt_type;            

            
            if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) { // K: Indicates that an advertisement packet received
                uint8_t *adv_data = scan_result->scan_rst.ble_adv;
                uint8_t adv_data_len = scan_result->scan_rst.adv_data_len;
                // k debug
                printf("K: Contents of adv_data: ");
                for (int i = 0; i < adv_data_len; i++){
                    printf("%02X", adv_data[i]);
                }
                printf("\n");

                // Variables to store data
                uint8_t flag_data[adv_data_len];
                uint8_t manufacturer_data[adv_data_len];
                int flag_data_length = 0;
                int manufacturer_data_length = 0;

                // Parse through the advertisement data to find the manufacturer-specific data
                for (int i = 0; i < adv_data_len;) {
                    uint8_t len = adv_data[i];
                    uint8_t type = adv_data[i + 1];
                    printf("K: i value is %d, len is 0x%02X, type is 0x%02x\n", i, len, type);

                    if (type == ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE && len >= 3) {
                        // printf("K: ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE reached \n");
                        // Check if the length is at least the minimum required for the manufacturer-specific data
                        uint16_t company_id = (adv_data[i + 3] << 8) | adv_data[i + 2];

                        if (company_id == 0x004C) { // Check for Apple device
                            // Store manufacturer-specific data
                            memcpy(manufacturer_data, adv_data + i, len + 1);
                            manufacturer_data_length = len + 1;
                        }                        

                    }
                    else if (type == ESP_BLE_AD_TYPE_FLAG) {
                        // Print the flag data
                        // printf("K: ESP_BLE_AD_TYPE_FLAG reached \n");
                        memcpy(flag_data, adv_data + i, len + 1);
                        flag_data_length = len + 1;
                    }
                    i += len + 1;
                }

                if (manufacturer_data_length > 0) {
                    printf("K: Apple BLE packet found\n");
                    // 09 Type
                    if (memcmp(manufacturer_data + 1, "\xFF\x4C\x00\x09", 4) == 0) {
                        print_manufacturer_data(bd_addr, manufacturer_data_length, manufacturer_data, flag_data_length, flag_data);
                        bool is_unique = true;
                        for (int i = 0; i < unique_macs_09_count; i++) {
                            if (memcmp(bd_addr, unique_macs_09[i], sizeof(bd_addr)) == 0) {
                                is_unique = false;
                                unique_macs_09_packet_count[i] += 1;
                                break;
                            }
                        }

                        if (is_unique) {
                            // MAC address not found, add it if space available
                            if (unique_macs_09_count < MAX_UNIQUE_MACS) {
                                memcpy(unique_macs_09[unique_macs_09_count], bd_addr, sizeof(bd_addr));
                                unique_macs_09_count++;
                                unique_macs_09_packet_count[unique_macs_09_count] += 1;
                            } else {
                                printf("Maximum number of unique MACs reached.\n");
                            }
                        } 
                    }

                    // // 10 Type
                    else if (memcmp(manufacturer_data + 1, "\xFF\x4C\x00\x10", 4) == 0) {
                        print_manufacturer_data(bd_addr, manufacturer_data_length, manufacturer_data, flag_data_length, flag_data);
                        bool is_unique = true;
                        for (int i = 0; i < unique_macs_10_count; i++) {
                            if (memcmp(bd_addr, unique_macs_10[i], sizeof(bd_addr)) == 0) {
                                is_unique = false;
                                unique_macs_10_packet_count[i] += 1;
                                break;
                            }
                        }

                        if (is_unique) {
                            // MAC address not found, add it if space available
                            if (unique_macs_10_count < MAX_UNIQUE_MACS) {
                                memcpy(unique_macs_10[unique_macs_10_count], bd_addr, sizeof(bd_addr));
                                unique_macs_10_count++;
                                unique_macs_10_packet_count[unique_macs_10_count] += 1;
                            } else {
                                printf("Maximum number of unique MACs reached.\n");
                            }
                        }
                    }
                    

                    // 16 type
                    else if (memcmp(manufacturer_data, "\xFF\x4C\x00\x16", 4) == 0) {
                        print_manufacturer_data(bd_addr, manufacturer_data_length, manufacturer_data, flag_data_length, flag_data);
                        bool is_unique = true;
                        for (int i = 0; i < unique_macs_16_count; i++) {
                            if (memcmp(bd_addr, unique_macs_16[i], sizeof(bd_addr)) == 0) {
                                is_unique = false;
                                unique_macs_16_packet_count[i] += 1;
                                break;
                            }
                        }

                        if (is_unique) {
                            // MAC address not found, add it if space available
                            if (unique_macs_16_count < MAX_UNIQUE_MACS) {
                                memcpy(unique_macs_16[unique_macs_16_count], bd_addr, sizeof(bd_addr));
                                unique_macs_16_count++;
                                unique_macs_16_packet_count[unique_macs_16_count] += 1;
                            } else {
                                printf("Maximum number of unique MACs reached.\n");
                            }
                        }
                    }

                    apple_packet_count++;
                }
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
            printf("K: ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT called\n");
            printf("Scan stopped\n");
            printf("Apple BLE packets found: %d\n", apple_packet_count);
            break;
        }

        default:
            break;
    }
}




void app_main(void)
{

    // Init Flash and BT
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    
    esp_err_t ret;
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(LOG_TAG, "Bluetooth controller initialize failed\n");
        return;
    }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(LOG_TAG, "Bluetooth controller enable failed\n");
        return;
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(LOG_TAG, "Bluedroid init failed\n");
        return;
    }    
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(LOG_TAG, "Bluedroid enable failed\n");
        return;
    }
    


    if ((ret = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "gap register error: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(LOG_TAG, "Callback initialized");

    ret = esp_ble_gap_set_scan_params(&ble_scan_params); // Set scan param which will also trigger the scanning
    if (ret != ESP_OK) {
        printf("Set scan parameters failed\n");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(SCAN_TIME*2)); // wait while the scan occurs
    

    printf("\n\nunique_mac count is %d, %d, %d\n", unique_macs_09_count, unique_macs_10_count, unique_macs_16_count);
    printf("Number of MAC is %d, Number of iPhone is %d\n", unique_macs_09_count, unique_macs_10_count - 2 * unique_macs_09_count > 0 ? unique_macs_10_count - 2 * unique_macs_09_count : 0 );

    print_unique_mac();

   
}

