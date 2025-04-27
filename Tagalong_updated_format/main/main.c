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

static const char* LOG_TAG = "findmy_modem";
static const char* MY_LOG = "My debug:";

#define EXP_DELAY 0 // If we want to introduce delay between experiment runs
#define EXP_MSG_TOTAL 50 // Define the number of messages for each experiment run. E.g. If NUM_MESSAGE = 300 and EXP_MSG_TOTAL = 50. 6 runs will occur

// Advertising related
#define INTERVAL_TIME 0x0020 // units of 0.625ms. 0x20 * 0.625ms = 20ms, Apple's recommendation
#define SEND_TIME 1000 // Used for the TaskDelay so this units is in ms
#define STATE_TTL 0x04 // Experimenting with using the state field in the adv_data as the TTL

// Set custom modem id before flashing:
static uint32_t modem_id = 0x693647dc;

static uint8_t data_to_send[] = {0x21, 0x00}; // end with 0x00 
const int NUM_MESSAGES = 300;
const int REPEAT_MESSAGE_TIMES = 1;
const int MESSAGE_DELAY = 0;


// Scan related
#define MAX_UNIQUE_MACS 1000
#define MIN_PACKET_COUNT 0
#define SCAN_TIME 3000 // in ms
static int applePacketCount = 0;

static esp_bd_addr_t unique_macs_09[MAX_UNIQUE_MACS];
static esp_bd_addr_t unique_macs_10[MAX_UNIQUE_MACS];
static esp_bd_addr_t unique_macs_16[MAX_UNIQUE_MACS];

static int unique_macs_09_packet_count[MAX_UNIQUE_MACS] = {0};
static int unique_macs_10_packet_count[MAX_UNIQUE_MACS] = {0};
static int unique_macs_16_packet_count[MAX_UNIQUE_MACS] = {0};

static int unique_macs_09_count = 0;
static int unique_macs_10_count = 0;
static int unique_macs_16_count = 0;

SemaphoreHandle_t semaphore; // For main to know when to resume


void log_current_unix_time() {
    time_t current_time;
    time(&current_time);

    ESP_LOGI("TIMESTAMP", "Unix: %ld", (long) current_time);
}
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

/** Random device address */
static esp_bd_addr_t rnd_addr = { 0xFF, 0xBB, 0xCB, 0xDD, 0xEE, 0xFF };

/** Advertisement payload */
static uint8_t adv_data[31] = {
    0x1e, /* Length (30) */
    0xff, /* Manufacturer Specific Data (type 0xff) */
    0x4c, 0x00, /* Company ID (Apple) */
    0x12, 0x19, /* Offline Finding type and length */
    STATE_TTL, /* State */
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

uint32_t swap_uint32( uint32_t val )
{
    val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF );
    return (val << 16) | (val >> 16);
};

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
    .scan_interval          = 0x4000, // 0x40 == 0.04 // 0x640 == 1s scan interval // originally 0x50
    .scan_window            = 0x4000 // 0x30 == 0.030s // 0x20 == 0.020s scan interval // originally 0x30
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
                printf("Scan parameters set, start scanning for %d ms...\n", SCAN_TIME); 
                esp_ble_gap_start_scanning(60); // let it scan indefinitely then stop later. To allow ms precision as the arg is in s.

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
            vTaskDelay(pdMS_TO_TICKS(SCAN_TIME)); // need to manually stop scan if ms precision is required.
            esp_ble_gap_stop_scanning();
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
                            // printf("K: Contents of adv_data: ");
                            // for (int i = 0; i < adv_data_len; i++){
                            //     printf("%02X", adv_data[i]);
                            // }
                            // printf("\n");
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
                    // 09 Type
                    if (memcmp(manufacturer_data + 1, "\xFF\x4C\x00\x09", 4) == 0) {
                        // print_manufacturer_data(bd_addr, manufacturer_data_length, manufacturer_data, flag_data_length, flag_data);
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
                        // print_manufacturer_data(bd_addr, manufacturer_data_length, manufacturer_data, flag_data_length, flag_data);
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

                    applePacketCount++;
                }
            }
            break;
        }
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT: {
            printf("K: ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT called\n");
            printf("Scan stopped\n");
            //printf("Apple BLE packets found: %d\n", applePacketCount);
            xSemaphoreGive(semaphore);

            break;
        }

        default:
            break;
    }
}

int is_valid_pubkey(uint8_t *pub_key_compressed) {
   uint8_t with_sign_byte[29];
   uint8_t pub_key_uncompressed[128];
   const struct uECC_Curve_t * curve = uECC_secp224r1();
   with_sign_byte[0] = 0x02;
   memcpy(&with_sign_byte[1], pub_key_compressed, 28);
   uECC_decompress(with_sign_byte, pub_key_uncompressed, curve);
   if(!uECC_valid_public_key(pub_key_uncompressed, curve)) {
       //ESP_LOGW(LOG_TAG, "Generated public key tested as invalid");
       return 0;
   }
   return 1;
}

void pub_from_priv(uint8_t *pub_compressed, uint8_t *priv) {
   const struct uECC_Curve_t * curve = uECC_secp224r1();
   uint8_t pub_key_tmp[128];
   uECC_compute_public_key(priv, pub_key_tmp, curve);
   uECC_compress(pub_key_tmp, pub_compressed, curve);
}

void set_addr_from_key(esp_bd_addr_t addr, uint8_t *public_key) {
    addr[0] = public_key[0] | 0b11000000;
    addr[1] = public_key[1];
    addr[2] = public_key[2]; 
    addr[3] = public_key[3];
    addr[4] = public_key[4];
    addr[5] = public_key[5];
}

void set_payload_from_key(uint8_t *payload, uint8_t *public_key) {
    // /* copy last 22 bytes */
    memcpy(&payload[7], &public_key[6], 22);
    /* append two bits of public key */
    payload[29] = public_key[0] >> 6;
}

void copy_4b_big_endian(uint8_t *dst, uint8_t *src) {
    dst[0] = src[3]; dst[1] = src[2]; dst[2] = src[1]; dst[3] = src[0];
}

void copy_2b_big_endian(uint8_t *dst, uint8_t *src) {
    dst[0] = src[1]; dst[1] = src[0];
}
void swap_modem_id_2MSB_message_id_2LSB(uint8_t* public_key) {
    uint8_t modem_id_2MSB[2] = {public_key[2], public_key[3]};
    uint8_t message_id_2LSB[2] = {public_key[10], public_key[11]};

    public_key[2] = message_id_2LSB[0]; public_key[3] = message_id_2LSB[1]; 
    public_key[10] = modem_id_2MSB[0]; public_key[11] = modem_id_2MSB[1];    
}
// Only works up to 8 bits per advert
// [2 byte magic] [4 byte modem_id] [2 byte tweak] [4 byte message id] [16 byte payload]
void set_addr_and_payload_for_byte(uint32_t index, uint32_t msg_id, uint8_t val, uint32_t chunk_len) {
    ESP_LOGI(MY_LOG, "Calling set_addr_and_payload_for_byte");
    
    uint16_t valid_key_counter = 0;
    static uint8_t public_key[28] = {0};
    public_key[0] = 0xBA; // magic value
    public_key[1] = 0xBE;
    copy_4b_big_endian(&public_key[2], &modem_id);
    public_key[6] = 0x00; /// init tweak value to 0
    public_key[7] = 0x00; /// init tweak value to 0
    copy_4b_big_endian(&public_key[8], &msg_id);
    
    // ! This is the 16 byte payload part. If index == 0 then just init it to all 0. Else, copy the last payload
    if (index) { 
        memcpy(&public_key[12], &curr_addr, 16);
    } else {
        memcpy(&public_key[12], &start_addr, 16);
    }

    uint32_t bit_index = index * chunk_len;
    uint32_t byte_index = bit_index/8;
    
    uint32_t start_byte = byte_index % 16;
    uint32_t next_byte = (start_byte + 1) % 16;
    
    uint32_t start_offset = bit_index % 8;
    
    /*!
        If chunk_len (the number of bits to encode) fits within a single byte, it XORs val into the byte at public_key[27 - start_byte].
        If the value spans two bytes, it splits the val across public_key[27 - start_byte] and public_key[27 - next_byte].
    */
    if ((8- start_offset) >= chunk_len) {
        // Fill only first bytes
        public_key[27 - start_byte] ^= val << start_offset;
    } else {
        // Fill both bytes
        public_key[27 - start_byte] ^= val << start_offset;
        public_key[27 - next_byte] ^= val >> (8 - start_offset);
    }

    memcpy(&curr_addr, &public_key[12], 16);

    // [2 byte magic] [2LSB message id] [2LSB modem_id] [2 byte tweak] [2 MSB message id] [2MSB modem_id] [16 byte payload]
    // To ensure different MAC address with each message
    swap_modem_id_2MSB_message_id_2LSB(public_key);

    do {
      copy_2b_big_endian(&public_key[6], &valid_key_counter);
	    valid_key_counter++;
    } while (!is_valid_pubkey(public_key));

    ESP_LOGI(LOG_TAG, "  pub key to use (%d. try): %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", valid_key_counter, public_key[0], public_key[1], public_key[2], public_key[3], public_key[4], public_key[5], public_key[6], public_key[7], public_key[8], public_key[9], public_key[10], public_key[11], public_key[12], public_key[13],public_key[14], public_key[15],public_key[16],public_key[17],public_key[18], public_key[19], public_key[20], public_key[21], public_key[22], public_key[23], public_key[24], public_key[25], public_key[26],  public_key[27]);

    set_addr_from_key(rnd_addr, public_key);
    set_payload_from_key(adv_data, public_key);
}

void advertising_cycle() {
    esp_err_t status;

    if ((status = esp_ble_gap_set_rand_addr(rnd_addr)) != ESP_OK) {
        ESP_LOGE(LOG_TAG, "couldn't set random address: %s", esp_err_to_name(status));
        return;
    }

    ESP_LOGI(MY_LOG, "Before calling esp_ble_gap_config_adv_data_raw");
    if ((esp_ble_gap_config_adv_data_raw((uint8_t*)&adv_data, sizeof(adv_data))) != ESP_OK) { // Triggers SET_COMPLETE callback
        ESP_LOGE(LOG_TAG, "couldn't configure BLE adv: %s", esp_err_to_name(status));
        return;
    }
    ESP_LOGI(MY_LOG, "After calling esp_ble_gap_config_adv_data_raw");
    xSemaphoreTake(semaphore, portMAX_DELAY); 
    ESP_LOGI(MY_LOG, "Ad cycle end");




}

void send_data_once_blocking(uint8_t* data_to_send, uint32_t len, uint32_t chunk_len, uint32_t msg_id) {
    ESP_LOGI(MY_LOG, "Calling send_data_once_blocking");

    uint32_t num_chunks = len * 8 / chunk_len;
    if (len * 8 % chunk_len) {
        num_chunks++;
    }
    
    for (uint32_t chunk_i = 0; chunk_i < num_chunks; chunk_i++) {       
        uint8_t chunk_value = 0;
        
        uint32_t start_bit = chunk_i * chunk_len;
        uint32_t end_bit = start_bit + chunk_len;
        
        // Calculate the byte and bit positions for start
        uint32_t start_byte = start_bit / 8;
        uint32_t start_bit_offset = start_bit % 8;
    
        // Number of bits to extract in the first byte
        uint32_t bits_in_first_byte = (8 - start_bit_offset) < chunk_len ? (8 - start_bit_offset) : chunk_len;
    
        // Extract bits from the start byte
        chunk_value = (data_to_send[start_byte] >> start_bit_offset) & ((1U << bits_in_first_byte) - 1);
    
        // Remaining bits to extract
        uint32_t remaining_bits = chunk_len - bits_in_first_byte;
    
        // Extract bits from the subsequent bytes
        while (remaining_bits > 0) {
            start_byte++;
            uint32_t bits_to_extract = (remaining_bits < 8) ? remaining_bits : 8;
            chunk_value |= ((uint32_t)data_to_send[start_byte] & ((1U << bits_to_extract) - 1)) << bits_in_first_byte;
            bits_in_first_byte += bits_to_extract;
            remaining_bits -= bits_to_extract;
        }

        set_addr_and_payload_for_byte(chunk_i, msg_id, chunk_value, chunk_len);
        log_current_unix_time();
        ESP_LOGD(LOG_TAG, "    Starting advertising cycle. Will now use device address: %02x %02x %02x %02x %02x %02x", rnd_addr[0], rnd_addr[1], rnd_addr[2], rnd_addr[3], rnd_addr[4], rnd_addr[5]);
        advertising_cycle();
        ESP_LOGI(MY_LOG, "after advertising_cycle called");

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
    
    semaphore = xSemaphoreCreateBinary(); // starts empty. Need to give before it can take

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

     // to wait for scan to stop before continueing.
    xSemaphoreTake(semaphore, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(2000)); // 2s delay to ensure processing scanned packets completed


    // If we want to ensure that mule is nearby before we start sending
    // If no unique mac detected during initial scanning, then just wait and repeat scan again
    // while (unique_macs_09_count == 0 && unique_macs_10_count == 0 && unique_macs_16_count == 0) {
    //     vTaskDelay(pdMS_TO_TICKS(5000));
    //     printf("No mules found, wait for 5s then scan again\n");
        
    //     ret = esp_ble_gap_set_scan_params(&ble_scan_params); // Set scan param which will also trigger the scanning
    //     if (ret != ESP_OK) {
    //         printf("Set scan parameters failed\n");
    //         return;
    //     } 
    //      xSemaphoreTake(semaphore, portMAX_DELAY);
    //     vTaskDelay(pdMS_TO_TICKS(2000)); // 2s delay to ensure processing scanned packets completed
    // }
    printf("unique_mac count is %d, %d, %d\n", unique_macs_09_count, unique_macs_10_count, unique_macs_16_count);
    // printf("Number of MAC is %d, Number of iPhone is %d\n", unique_macs_09_count, unique_macs_10_count - 2 * unique_macs_09_count > 0 ? unique_macs_10_count - 2 * unique_macs_09_count : 0 );
    
    // Send Message
    uint32_t current_message_id = 0;

    printf("Bytes: ");
    for (int i = 0; i < sizeof(data_to_send); i++) {
        printf("%02x ", data_to_send[i]);
    }
    printf("\n");
    
    for (uint32_t i = 0; i < NUM_MESSAGES; i++) {
        // generateAlphaSequence(i, data_to_send);
        for (int j = 0; j < REPEAT_MESSAGE_TIMES; j++) {
            unique_macs_09_count = 0; unique_macs_10_count = 0; unique_macs_16_count = 0;

            // For scanning between messages (during experiments with long delay to ensure isolation maintained)
            // ret = esp_ble_gap_set_scan_params(&ble_scan_params); // Set scan param which will also trigger the scanning
            // if (ret != ESP_OK) {
            //     printf("Set scan parameters failed\n");
            //     return;
            // } 
            // xSemaphoreTake(semaphore, portMAX_DELAY);
            // vTaskDelay(pdMS_TO_TICKS(2000)); // 2s delay to ensure processing scanned packets completed
            // printf("unique_mac count is %d, %d, %d\n", unique_macs_09_count, unique_macs_10_count, unique_macs_16_count);

            send_data_once_blocking(data_to_send, sizeof(data_to_send) - 1, 8, current_message_id);
            // vTaskDelay(MESSAGE_DELAY);
        }

        current_message_id++;
        vTaskDelay(pdMS_TO_TICKS(MESSAGE_DELAY));
        // data_to_send[0]++;

        if (current_message_id == EXP_MSG_TOTAL) {
            current_message_id = 0;
            
            // Delay between experiment run
            vTaskDelay(pdMS_TO_TICKS(EXP_DELAY));
       
            modem_id++;
        }

    }

    // Wrap up and end
    log_current_unix_time();
    esp_wifi_disconnect();
    esp_wifi_stop();
}
