#ifndef TASKSDEFINE_H
#define TASKSDEFINE_H

#define SLEEP_ENABLED // do we sleep in the end of loop
#define DEBUG_ENABLED // debug print
#undef DEVICE_SCAN_WIFI_0_ENABLED //PAX scan
#undef DEVICE_SCAN_BLE_1_ENABLED
#define SEND_DATA_LORA_2_ENABLED
#undef SEND_DATA_WIFI_3_ENABLED
#undef OTA_ENABLED // Will not work with sleep
#undef READ_TEMP_HUM_BME280_5_ENABLED
#undef READ_HTU21D_6_ENABLED
#undef SYNCRONIZE_NTP_TIME_7_ENABLED
#undef READ_WEATHER_DAVIS_8_ENABLED
#undef READ_EXTERNAL_VOLTAGE_9_ENABLED
#define READ_VICTRON_ENABLED
#undef READ_ACUDC
#undef RESTART_10_ENABLED

#endif