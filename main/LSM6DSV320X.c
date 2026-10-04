#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

/* Change these three values for your hotspot and receiving computer. */
#define WIFI_SSID       "iPhone (177)"
#define WIFI_PASSWORD   "Z65hry#6(;&"
#define RECEIVER_IP     ""
#define RECEIVER_PORT   5000

/* ESP32 SPI pin mapping. */
#define PIN_MOSI        23
#define PIN_MISO        19
#define PIN_LSM_CS      5
#define PIN_CLK         18

/* LSM6DSV320X register addresses. */
#define CTRL1           0x10
#define CTRL2           0x11
#define CTRL6           0x15
#define WHO_AM_I        0x0F
#define OUTX_L_G        0x22
#define OUTY_L_G        0x24
#define OUTZ_L_G        0x26
#define OUTX_L_A        0x28
#define OUTY_L_A        0x2A
#define OUTZ_L_A        0x2C

/* Conversion values for the configured sensor ranges. */
#define ACCEL_SENS_MG_PER_LSB   0.061f
#define G_TO_MS2                9.80665f
#define GYRO_SENS_MDPS_PER_LSB  17.5f
#define ACCEL_MODE              1

#define WIFI_CONNECTED_BIT BIT0

static const char *TAG = "imu_udp";
static EventGroupHandle_t wifi_event_group;

static esp_err_t reg_write(spi_device_handle_t dev,
                           uint8_t reg_addr,
                           uint8_t data)
{
    spi_transaction_t transaction = {
        .cmd = 0,
        .addr = reg_addr,
        .length = 8,
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {data},
    };

    return spi_device_polling_transmit(dev, &transaction);
}

static esp_err_t reg_read(spi_device_handle_t dev,
                          uint8_t reg_addr,
                          uint8_t *data)
{
    spi_transaction_t transaction = {
        .cmd = 1,
        .addr = reg_addr,
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    esp_err_t err = spi_device_polling_transmit(dev, &transaction);
    if (err == ESP_OK) {
        *data = transaction.rx_data[0];
    }

    return err;
}

static int16_t read_imu_axis(spi_device_handle_t dev, uint8_t low_reg)
{
    uint8_t low = 0;
    uint8_t high = 0;

    ESP_ERROR_CHECK(reg_read(dev, low_reg, &low));
    ESP_ERROR_CHECK(reg_read(dev, low_reg + 1, &high));

    return (int16_t)(((uint16_t)high << 8) | low);
}

static void configure_imu(spi_device_handle_t dev)
{
    uint8_t accel_control = (ACCEL_MODE << 4) | 6; /* 0x16 */
    uint8_t gyro_dsp_control = (1 << 4) | 10;      /* 0x1A */
    uint8_t gyro_control = (1 << 4) | 6;           /* 0x16 */

    ESP_ERROR_CHECK(reg_write(dev, CTRL1, accel_control));
    ESP_ERROR_CHECK(reg_write(dev, CTRL6, gyro_dsp_control));
    ESP_ERROR_CHECK(reg_write(dev, CTRL2, gyro_control));
}

static spi_device_handle_t initialize_spi(void)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = PIN_MOSI,
        .miso_io_num = PIN_MISO,
        .sclk_io_num = PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(
        SPI2_HOST,
        &bus_config,
        SPI_DMA_CH_AUTO));

    spi_device_interface_config_t device_config = {
        .clock_speed_hz = 10 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_LSM_CS,
        .queue_size = 7,
        .command_bits = 1,
        .address_bits = 7,
        .dummy_bits = 0,
    };

    spi_device_handle_t dev;
    ESP_ERROR_CHECK(spi_bus_add_device(
        SPI2_HOST,
        &device_config,
        &dev));

    return dev;
}

static void wifi_event_handler(void *arg,
                               esp_event_base_t event_base,
                               int32_t event_id,
                               void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_ERROR_CHECK(esp_wifi_connect());
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *event = event_data;
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "Wi-Fi disconnected (reason %d); reconnecting", event->reason);
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = event_data;
        ESP_LOGI(TAG, "ESP32 IP address: " IPSTR,
                 IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void initialize_wifi(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Could not create Wi-Fi event group");
        abort();
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_config));

    ESP_ERROR_CHECK(esp_event_handler_register(
        WIFI_EVENT,
        ESP_EVENT_ANY_ID,
        wifi_event_handler,
        NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(
        IP_EVENT,
        IP_EVENT_STA_GOT_IP,
        wifi_event_handler,
        NULL));

    wifi_config_t wifi_config = {0};
    snprintf((char *)wifi_config.sta.ssid,
             sizeof(wifi_config.sta.ssid),
             "%s",
             WIFI_SSID);
    snprintf((char *)wifi_config.sta.password,
             sizeof(wifi_config.sta.password),
             "%s",
             WIFI_PASSWORD);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Waiting to connect to hotspot '%s'", WIFI_SSID);
    xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        portMAX_DELAY);
}

static int initialize_udp(struct sockaddr_in *destination)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Could not create UDP socket: %s", strerror(errno));
        return -1;
    }

    memset(destination, 0, sizeof(*destination));
    destination->sin_family = AF_INET;
    destination->sin_port = htons(RECEIVER_PORT);

    if (inet_pton(AF_INET, RECEIVER_IP, &destination->sin_addr) != 1) {
        ESP_LOGE(TAG, "Invalid receiver IP address: %s", RECEIVER_IP);
        close(sock);
        return -1;
    }

    ESP_LOGI(TAG, "Sending UDP telemetry to %s:%d",
             RECEIVER_IP, RECEIVER_PORT);
    return sock;
}

void app_main(void)
{
    spi_device_handle_t imu = initialize_spi();

    uint8_t device_id = 0;
    ESP_ERROR_CHECK(reg_read(imu, WHO_AM_I, &device_id));
    ESP_LOGI(TAG, "LSM6DSV320X WHO_AM_I: 0x%02X", device_id);

    configure_imu(imu);
    initialize_wifi();

    struct sockaddr_in udp_destination;
    int udp_socket = initialize_udp(&udp_destination);

    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    float yaw_deg = 0.0f;
    int64_t previous_time_us = esp_timer_get_time();
    uint32_t sequence = 0;

    while (true) {
        int16_t accel_x_raw = read_imu_axis(imu, OUTX_L_A);
        int16_t accel_y_raw = read_imu_axis(imu, OUTY_L_A);
        int16_t accel_z_raw = read_imu_axis(imu, OUTZ_L_A);

        int16_t gyro_x_raw = read_imu_axis(imu, OUTX_L_G);
        int16_t gyro_y_raw = read_imu_axis(imu, OUTY_L_G);
        int16_t gyro_z_raw = read_imu_axis(imu, OUTZ_L_G);

        float accel_x_ms2 = accel_x_raw * ACCEL_SENS_MG_PER_LSB
                            / 1000.0f * G_TO_MS2;
        float accel_y_ms2 = accel_y_raw * ACCEL_SENS_MG_PER_LSB
                            / 1000.0f * G_TO_MS2;
        float accel_z_ms2 = accel_z_raw * ACCEL_SENS_MG_PER_LSB
                            / 1000.0f * G_TO_MS2;

        float gyro_x_dps = gyro_x_raw * GYRO_SENS_MDPS_PER_LSB / 1000.0f;
        float gyro_y_dps = gyro_y_raw * GYRO_SENS_MDPS_PER_LSB / 1000.0f;
        float gyro_z_dps = gyro_z_raw * GYRO_SENS_MDPS_PER_LSB / 1000.0f;

        int64_t now_us = esp_timer_get_time();
        float elapsed_seconds = (now_us - previous_time_us) / 1000000.0f;
        previous_time_us = now_us;

        pitch_deg += gyro_x_dps * elapsed_seconds;
        roll_deg += gyro_y_dps * elapsed_seconds;
        yaw_deg += gyro_z_dps * elapsed_seconds;

        char packet[256];
        int packet_length = snprintf(
            packet,
            sizeof(packet),
            "%lu,%lld,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
            (unsigned long)sequence++,
            (long long)now_us,
            accel_x_ms2,
            accel_y_ms2,
            accel_z_ms2,
            gyro_x_dps,
            gyro_y_dps,
            gyro_z_dps,
            pitch_deg,
            roll_deg,
            yaw_deg);

        printf("%s", packet);

        if (udp_socket >= 0 &&
            (xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT)) {
            int sent = sendto(
                udp_socket,
                packet,
                packet_length,
                0,
                (struct sockaddr *)&udp_destination,
                sizeof(udp_destination));

            if (sent < 0) {
                ESP_LOGW(TAG, "UDP send failed: %s", strerror(errno));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
