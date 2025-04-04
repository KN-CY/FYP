# Tagalong_original_format
- Consists of the ESP32 Firmware that sends Find My BLE advertisements with the original Tagalong encoding format
- Consist of the Swift Datafetcher application to fetch and decode data in the original Tagalong format

# Taglong_updated_format
- Consists of the ESP32 Firmware that sends Find My BLE advertisements with the updated Tagalong encoding format
- Consist of the Swift Datafetcher application to fetch and decode data in the updated Tagalong format

# Echoer
- Consist of the ESP32 Firmware used in the experiments to test the relay network of IoT devices
- ESP32 will listen for Find My advertisements from other IoT devices and echo it. Also takes into consideration TTL

# ScanTimeTest
- ESP32 Firmware to listen for Apple BLE advertisements
- Useful for debugging and to estimate the number of Apple devices within BLE range
- Can help to ensure environment is isolated
- Can also use Android nRF Connect app 
