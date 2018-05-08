#define LOG_LOCAL_LEVEL ESP_LOG_INFO

#include <stdio.h>
#include <vl53l0x_platform.h>
#include "vl53l0x_helper.h"
#include "mqtt.h"

#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include <driver/i2c.h>
#include <vl53l0x_def.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"

static const char *TAG = "salt-level-sensor-vl53l0x";
static const int WIFI_CONNECTED_BIT = BIT0;
static const int MQTT_CONNECTED_BIT = BIT1;

static VL53L0X_Dev_t vl53l0x_dev;
static TimerHandle_t sensor_timer_h = NULL;
static EventGroupHandle_t connection_event_group;
static mqtt_client *mqtt_client_h = NULL;

static void read_sensor(TimerHandle_t timer_handle);
static void mqtt_on_connected(mqtt_client * client, mqtt_event_data_t * event_data)
{
    mqtt_client_h = client;
    xEventGroupSetBits(connection_event_group, MQTT_CONNECTED_BIT);
    ESP_LOGI(TAG, "MQTT Connected");
    mqtt_publish(client, CONFIG_MQTT_LWT_TOPIC, "online", 6, 1, 1);

    sensor_timer_h = xTimerCreate(
            "reading_timer",
            pdMS_TO_TICKS(CONFIG_SENSOR_READ_INTERVAL * 1000),
            true,
            NULL,
            read_sensor
    );

    if (!sensor_timer_h || (xTimerStart(sensor_timer_h, pdMS_TO_TICKS(1000)) == pdFAIL))
    {
        ESP_LOGE(TAG, "Failed to start timer");
        esp_restart();
    }

    read_sensor(NULL);
}

static void mqtt_on_disconnected(mqtt_client * client, mqtt_event_data_t * event_data)
{
    xEventGroupClearBits(connection_event_group, MQTT_CONNECTED_BIT);
    ESP_LOGW(TAG, "MQTT Disconnected");

    // Stop the sensor reading
    if(xTimerDelete(sensor_timer_h, pdMS_TO_TICKS(10000)) == pdFAIL)
    {
        esp_restart();
    }

    mqtt_client_h = NULL;

    if (!(xEventGroupGetBits(connection_event_group) & WIFI_CONNECTED_BIT))
    {
        ESP_ERROR_CHECK(esp_wifi_connect());
    }
}

mqtt_settings settings = {
        .host = CONFIG_MQTT_HOST,
        .username = CONFIG_MQTT_USERNAME,
        .password = CONFIG_MQTT_PASSWORD,
        .port = CONFIG_MQTT_PORT,
        .client_id = "salt_sensor",
        .lwt_topic = CONFIG_MQTT_LWT_TOPIC,
        .lwt_msg = "offline",
        .lwt_msg_len = 7,
        .lwt_qos = 1,
        .lwt_retain = 1,
        .clean_session = 1,
        .keepalive = 60,
        .auto_reconnect = true,
        .connected_cb = mqtt_on_connected,
        .disconnected_cb = mqtt_on_disconnected,
};

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(connection_event_group, WIFI_CONNECTED_BIT);

            // If still connected to MQTT, disconnect
            if (xEventGroupGetBits(connection_event_group) & MQTT_CONNECTED_BIT)
            {
                mqtt_stop();
                while (xEventGroupGetBits(connection_event_group) & MQTT_CONNECTED_BIT)
                {
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }

            if (!mqtt_start(&settings))
            {
                ESP_LOGE(TAG, "Failed to create mqtt client!");
                break;
            }

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            xEventGroupClearBits(connection_event_group, WIFI_CONNECTED_BIT);
            ESP_LOGW(TAG, "Disconnected from wifi");

            mqtt_stop();

            // Reconnect in mqtt_on_disconnected

            break;
        default:
            break;
    }
    return ESP_OK;
}

static void init_wifi()
{
    tcpip_adapter_init();
    connection_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
            .sta = {
                    .ssid = CONFIG_WIFI_SSID,
                    .password = CONFIG_WIFI_PASSWORD,
                    .bssid_set = false,
            }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );
}

static void i2c_master_init()
{
    i2c_port_t i2c_master_port = I2C_NUM_1;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = GPIO_NUM_18;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_io_num = GPIO_NUM_19;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = 400000;

    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0));
}

static void read_sensor(TimerHandle_t timer_handle)
{
    uint16_t vals[CONFIG_SENSOR_SAMPLE_COUNT];
    char json_buffer[64];
    VL53L0X_Error status;
    VL53L0X_RangingMeasurementData_t measurement_data;
    uint32_t reading = 0;
    uint8_t percent_full = 0;

    for (int i = 0; i < CONFIG_SENSOR_SAMPLE_COUNT; i++)
    {
        status = take_reading(&vl53l0x_dev, &measurement_data);
        if (status != VL53L0X_ERROR_NONE)
            esp_restart();

        vals[i] = measurement_data.RangeMilliMeter;
        reading += vals[i];
    }

    reading /= CONFIG_SENSOR_SAMPLE_COUNT;

    if (reading <= CONFIG_SENSOR_FULL_LVL_MM)
    {
        percent_full = 100;
    }
    else if (reading >= CONFIG_SENSOR_EMPTY_LVL_MM)
    {
        percent_full = 0;
    }
    else
    {
        percent_full = (uint8_t) (100 - ((reading - CONFIG_SENSOR_FULL_LVL_MM) / (float)(CONFIG_SENSOR_EMPTY_LVL_MM - CONFIG_SENSOR_FULL_LVL_MM) * 100));
    }

    int msg_size = sprintf(json_buffer, "{\"reading\":%u,\"percent_full\":%d}", reading, percent_full);

    ESP_LOGI(TAG, "Measured distance: %i", reading);
    mqtt_publish(mqtt_client_h, CONFIG_MQTT_SALT_LEVEL_TOPIC, json_buffer, msg_size, 0, 1);
}

void app_main()
{
    nvs_flash_init();
    i2c_master_init();

    vl53l0x_dev.i2c_port_num = I2C_NUM_1;
    vl53l0x_dev.i2c_address = 0x29;

    VL53L0X_Error status = init_vl53l0x(&vl53l0x_dev);

    if (status != VL53L0X_ERROR_NONE)
        esp_restart();

    init_wifi();
}
