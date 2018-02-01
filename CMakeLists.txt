cmake_minimum_required(VERSION 3.9)
project(esp32_salt_level_sensor_2)

set(IDF_PATH /home/khendricks/dev/esp32/tools/esp-idf)

include_directories(
        build/include
        components/esp32-vl53l0x/core/inc
        components/esp32-vl53l0x/platform/inc

        components/esp32mqtt/include

        ${IDF_PATH}/components/log/include
        ${IDF_PATH}/components/newlib/include
        ${IDF_PATH}/components/esp32/include
        ${IDF_PATH}/components/driver/include
        ${IDF_PATH}/components/freertos/include
        ${IDF_PATH}/components/tcpip_adapter/include
        ${IDF_PATH}/components/nvs_flash/include
        ${IDF_PATH}/components/soc/esp32/include
)

add_executable(esp32_salt_level_sensor_2
        components/esp32mqtt/mqtt.c
        components/esp32mqtt/mqtt_msg.c
        components/esp32mqtt/ringbuf.c
        components/esp32-vl53l0x/core/src/vl53l0x_api.c
        components/esp32-vl53l0x/core/src/vl53l0x_api_calibration.c
        components/esp32-vl53l0x/core/src/vl53l0x_api_core.c
        components/esp32-vl53l0x/core/src/vl53l0x_api_ranging.c
        components/esp32-vl53l0x/core/src/vl53l0x_api_strings.c
        components/esp32-vl53l0x/platform/src/vl53l0x_platform.c
        main/main.c
        main/vl53l0x_helper.c
        main/vl53l0x_helper.h
        build/include/sdkconfig.h
)
