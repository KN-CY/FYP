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

// Advertising related
#define INTERVAL_TIME 0x0020 // units of 0.625ms. 0x20 * 0.625ms = 20ms, Apple's recommendation
#define SEND_TIME 1000 // Used for the TaskDelay so this units is in ms

// Scan related
#define MAX_UNIQUE_MACS 500
#define MIN_PACKET_COUNT 0
#define SCAN_TIME 300000 // in ms
static int apple_packet_count = 0;

// unique MAC as an identifier to each unique FindMy message
static esp_bd_addr_t unique_macs[MAX_UNIQUE_MACS];
// bool is_sent[MAX_UNIQUE_MACS] = {false}; // Not necessary after introducing TTL

static int unique_macs_count = 0;

// adv_data is at most 31 bytes long
#define ADV_DATA_LEN 31
static uint8_t adv_data_storage[MAX_UNIQUE_MACS][ADV_DATA_LEN] = {0};

SemaphoreHandle_t semaphore; // For main to know when to resume

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




/** Callback function for BT events */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

/* https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_gap_ble.html#_CPPv420esp_ble_adv_params_t */
static esp_ble_adv_params_t ble_adv_params = {
    // Advertising min interval:
    // Minimum advertising interval for undirected and low duty cycle
    // directed advertising. Range: 0x0020 to 0x4000 Default: N = 0x0800
    // (1.28 second) Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec
    .adv_int_min        = INTERVAL_TIME, 
    // Advertising max interval:
    // Maximum advertising interval for undirected and low duty cycle
    // directed advertising. Range: 0x0020 to 0x4000 Default: N = 0x0800
    // (1.28 second) Time = N * 0.625 msec Time Range: 20 ms to 10.24 sec
    .adv_int_max        = INTERVAL_TIME, 
    // Advertisement type
    .adv_type           = ADV_TYPE_NONCONN_IND,
    // Use the random address
    .own_addr_type      = BLE_ADDR_TYPE_RANDOM,
    // All channels
    .channel_map        = ADV_CHNL_ALL,
    // Allow both scan and connection requests from anyone. 
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE, // originally  BLE_SCAN_TYPE_ACTIVE
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50, // 0x4000 > 10s // 0x1e0, //0x30, // 0x40 == 0.04 // 0x640 == 1s scan interval // originally 0x50
    .scan_window            = 0x40 // 0x1e0 //0x30 // 0x30 == 0.030s // 0x20 == 0.020s scan interval // originally 0x30
    // .scan_interval          = 0x4000, // 0x4000 > 10s // 0x1e0, //0x30, // 0x40 == 0.04 // 0x640 == 1s scan interval // originally 0x50
    // .scan_window            = 0x4000 // 0x1e0 //0x30 // 0x30 == 0.030s // 0x20 == 0.020s scan interval // originally 0x30
};// param definitions: https://github.com/pycom/esp-idf-2.0/blob/master/components/bt/bluedroid/api/include/esp_gap_ble_api.h#L203

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    esp_err_t err;

    switch (event) {
        // Advertising related
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT: /*!< When raw advertising data set complete, the event comes */
            ESP_LOGI(MY_LOG, "Case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT reached");
            // ESP_LOGI(MY_LOG, "Before calling esp_ble_gap_start_advertising");
            esp_ble_gap_start_advertising(&ble_adv_params);
            // ESP_LOGI(MY_LOG, "After calling esp_ble_gap_start_advertising");

            break;

        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: /*!< When start advertising complete, the event comes */
            ESP_LOGI(MY_LOG, "Case ESP_GAP_BLE_ADV_START_COMPLETE_EVT reached");
            // is it running?
            if ((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(LOG_TAG, "advertising start failed: %s", esp_err_to_name(err));
            } else {
                ESP_LOGI(LOG_TAG, "advertising started");
                vTaskDelay(pdMS_TO_TICKS(SEND_TIME));
                // printf("pdMS_TO_TICKS(SEND_TIME*10) is %ld\n", pdMS_TO_TICKS(SEND_TIME*10));
                esp_ble_gap_stop_advertising();
            }

            // esp_ble_gap_stop_advertising(); // testing out sending just 1 packet

            
            break;

        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: /*!< When stop adv complete, the event comes */
            ESP_LOGI(MY_LOG, "Case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT reached");
            if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
                ESP_LOGE(LOG_TAG, "adv stop failed: %s", esp_err_to_name(err));
            }
            else {
                ESP_LOGI(LOG_TAG, "advertising stopped");
                xSemaphoreGive(semaphore);
            }
            break;
        // Scan related
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
            // printf("K: ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT called\n");
            if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                // printf("Scan parameters set, start scan cycle for %d ms...\n", 1000);
                esp_ble_gap_start_scanning(0); // let it scan indefinitely then stop later. To allow ms precision as the arg is in s.
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
            // we let each cycle scan for just 1000ms. 
            // For some reason (maybe because of memory) if we let 1 cycle be very long it can at most get 4-5 unique Find My messages
            vTaskDelay(pdMS_TO_TICKS(1000)); // need to manually stop scan if ms precision is required.
            esp_ble_gap_stop_scanning();
            break;
        }
        case ESP_GAP_BLE_SCAN_RESULT_EVT: {
            // printf("K: ESP_GAP_BLE_SCAN_RESULT_EVT called\n");
            esp_ble_gap_cb_param_t *scan_result = param;

            esp_bd_addr_t bd_addr; // K: Stores the mac address of the BLE device
            memcpy(bd_addr, scan_result->scan_rst.bda, sizeof(bd_addr));

            // esp_bt_dev_type_t dev_type = scan_result->scan_rst.dev_type ; // Flag for device capability
            // esp_ble_addr_type_t add_type = scan_result->scan_rst.ble_addr_type;
            // esp_ble_evt_type_t event_type = scan_result->scan_rst.ble_evt_type;            

            
            if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) { // K: Indicates that an advertisement packet received
                uint8_t *adv_data = scan_result->scan_rst.ble_adv;
                uint8_t adv_data_len = scan_result->scan_rst.adv_data_len;
                // k debug
                // printf("K: Contents of adv_data: ");
                // for (int i = 0; i < adv_data_len; i++){
                //     printf("%02X", adv_data[i]);
                // }
                // printf("\n");

                // Variables to store data
                uint8_t flag_data[adv_data_len];
                uint8_t manufacturer_data[adv_data_len];
                int flag_data_length = 0;
                int manufacturer_data_length = 0;

                // Parse through the advertisement data to find the manufacturer-specific data
                for (int i = 0; i < adv_data_len;) {
                    uint8_t len = adv_data[i];
                    uint8_t type = adv_data[i + 1];
                    // printf("K: i value is %d, len is 0x%02X, type is 0x%02x\n", i, len, type);

                    

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
                    // printf("K: Apple BLE packet found\n");
         
                    // print_manufacturer_data(bd_addr, manufacturer_data_length, manufacturer_data, flag_data_length, flag_data);
                    
                   
                    // 12 type. Find My message detected
                    // 19 is the apple payload length. The find my message with the public key will have the length as 0x19
                    if (memcmp(manufacturer_data + 1, "\xFF\x4C\x00\x12\x19", 5) == 0) {
                        printf("Find My message detected\n");
                        // print_manufacturer_data(bd_addr, manufacturer_data_length, manufacturer_data, flag_data_length, flag_data);
                        
                        // Check status and make sure TTL has not expired
                        bool is_expired_TTL = false;
                        if (adv_data[6] == 0x00) {
                            printf("TTL exceeded\n");
                            is_expired_TTL = true;
                        }

                         // Ensure that Modem_id address matches our IoT device
                         bool is_our_device = true;
                         if (bd_addr[4] != 0xff) {
                            printf("Not our device's Find My message\n");
                            is_our_device = false;
                        }
                        
                        bool is_unique = true;
                        for (int i = 0; i < unique_macs_count; i++) {
                            if (memcmp(bd_addr, unique_macs[i], sizeof(bd_addr)) == 0) {
                                is_unique = false;
                                break;
                            }
                        }

                        if (is_unique && !is_expired_TTL && is_our_device) {
                            printf("is_unique MAC and TTL not expired and packet belongs to us\n");

                            adv_data[6]--; // decrease the TTL
                            
                           

                            // MAC address not found and TTL not exceeded, add it if space available
                            if (unique_macs_count < MAX_UNIQUE_MACS) {
                                memcpy(unique_macs[unique_macs_count], bd_addr, sizeof(bd_addr));
                                memcpy(adv_data_storage[unique_macs_count], adv_data, adv_data_len);
                                printf("Added new Find My message to echo later\n");
                                unique_macs_count++;
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
            xSemaphoreGive(semaphore);
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
    
    semaphore = xSemaphoreCreateBinary(); // starts empty. Need to give before it can take

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


    // Just beacon some BLE advertisement for sanity check, so we can use nrf connect to make sure this device is alive
    esp_bd_addr_t beacon_mac = {0xff, 0xee, 0xdd, 0xcc, 0xbb, 0xaa};

    esp_err_t status;
    if ((status = esp_ble_gap_set_rand_addr(beacon_mac)) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "couldn't set random address: %s", esp_err_to_name(status));
        return;

    }

    // uint8_t beacon_adv_data[ADV_DATA_LEN] = {};

    // if ((esp_ble_gap_config_adv_data_raw((uint8_t*)&beacon_adv_data, sizeof(beacon_adv_data))) != ESP_OK) { // Triggers SET_COMPLETE callback
    //     ESP_LOGE(LOG_TAG, "couldn't configure BLE adv: %s", esp_err_to_name(status));
    //     return;
    // }
    // vTaskDelay(pdMS_TO_TICKS(2000));

    
    // for (int i = 0; i < 10; i++) {
    // endlessly loop the cycle of scanning and echoing
    for (;;) {
       
        // Each Scan cycle is 1000ms. e.g. If SCAN_TIME = 10000, then 10 cycles are done
        // For some reason (maybe memory) we cannot have a scan cycle be too long or else we can get only max 4-5 unique Find My Messages
        
        printf("Scanning for %dms, scanning for %d cycles of 1000 ms each...\n", SCAN_TIME, SCAN_TIME/1000);
        for (int j = 0; j < SCAN_TIME; j += 1000){
            ret = esp_ble_gap_set_scan_params(&ble_scan_params); // Set scan param which will also trigger the scanning
            if (ret != ESP_OK) {
                printf("Set scan parameters failed\n");
                return;
            }
            // to wait for scan to stop before continueing.
            xSemaphoreTake(semaphore, portMAX_DELAY); 
        }
        // unique mac of Find My messages
        printf("unique mac_count is %d\n", unique_macs_count);

        // Advertise the messages
        for (uint32_t i = 0; i < unique_macs_count; i++) {
            esp_bd_addr_t curr_mac;
            memcpy(curr_mac, unique_macs[i], ESP_BD_ADDR_LEN);
            uint8_t curr_adv_data[ADV_DATA_LEN];
            memcpy(curr_adv_data, adv_data_storage[i], ADV_DATA_LEN);
            printf("MAC Address: ");
            for (int j = 0; j < ESP_BD_ADDR_LEN; j++) {
                printf("%02X:", curr_mac[j]);
            }
            printf(" | ");
            printf("Adv data: ");
            for (int j = 0; j < ADV_DATA_LEN; j++) {
                printf("%02X ", curr_adv_data[j]);
            }
            printf("\n");
            
  
            printf("Echoing message: \n");

            esp_err_t status;
            if ((status = esp_ble_gap_set_rand_addr(curr_mac)) != ESP_OK) {
                ESP_LOGE(LOG_TAG, "couldn't set random address: %s", esp_err_to_name(status));
                return;
            }
            if ((esp_ble_gap_config_adv_data_raw((uint8_t*)&curr_adv_data, sizeof(curr_adv_data))) != ESP_OK) { // Triggers SET_COMPLETE callback
                ESP_LOGE(LOG_TAG, "couldn't configure BLE adv: %s", esp_err_to_name(status));
                return;
            }
            xSemaphoreTake(semaphore, portMAX_DELAY);       
        }

        // Clear list of messages and MAC address
        unique_macs_count = 0;


    }
}

