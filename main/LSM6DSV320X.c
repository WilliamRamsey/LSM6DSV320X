#include <stdio.h>
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// PIN MAPPING
#define MOSI 23
#define MISO 19
#define LSM_CS 5
#define CLK 18

// IMU REGISTER ADDRESSES
# define CTRL1 0x10    // Accel Control Register
# define CTRL2 0x11    // Gyro Control Register
# define CTRL6 0x15    // Gyro full-scale range (+-250 to +-4000 dps (deg/sec) and low pass filter)
# define OUTX_L_A 0x28 // Accel X-axis, low byte  (bits 7:0)
# define OUTX_H_A 0x29 // Accel X-axis, high byte (bits 15:8)
# define OUTY_L_A 0x2A // Accel Y-axis, low byte  (bits 7:0)
# define OUTY_H_A 0x2B // Accel Y-axis, high byte (bits 15:8)
# define OUTZ_L_A 0x2C // Accel Z-axis, low byte  (bits 7:0)
# define OUTZ_H_A 0x2D // Accel Z-axis, high byte (bits 15:8)
# define OUTX_L_G 0x22 // Gyro X-axis (pitch), low byte  (bits 7:0)
# define OUTX_H_G 0x23 // Gyro X-axis (pitch), high byte (bits 15:8)
# define OUTY_L_G 0x24 // Gyro Y-axis (roll),  low byte  (bits 7:0)
# define OUTY_H_G 0x25 // Gyro Y-axis (roll),  high byte (bits 15:8)
# define OUTZ_L_G 0x26 // Gyro Z-axis (yaw),   low byte  (bits 7:0)
# define OUTZ_H_G 0x27 // Gyro Z-axis (yaw),   high byte (bits 15:8)
# define SFLP_GRAVX_L 0x1E // Gravity X-axis, low byte  (bits 7:0)
# define SFLP_GRAVX_H 0x1F // Gravity X-axis, high byte (bits 15:8)
# define SFLP_GRAVY_L 0x20 // Gravity Y-axis, low byte  (bits 7:0)
# define SFLP_GRAVY_H 0x21 // Gravity Y-axis, high byte (bits 15:8)
# define SFLP_GRAVZ_L 0x22 // Gravity Z-axis, low byte  (bits 7:0)
# define SFLP_GRAVZ_H 0x23 // Gravity Z-axis, high byte (bits 15:8)

// BANK switching register
#define FUNC_CFG_ACCESS 0x01
#define EMB_FUNC_REG_ACCESS  (1 << 7)

// Embedded functions configuration registers
#define EMB_FUNC_EN_A 0x04
#define SFLP_GAME_EN (1 << 3)


// ±2 g full scale (CTRL8 default): 0.061 mg/LSB
#define ACCEL_SENS_MG_PER_LSB 0.061f
#define G_TO_MS2 9.80665f

// ±500 dps full scale (CTRL6 = 0x1A): 17.5 mdps/LSB
#define GYRO_SENS_MDPS_PER_LSB 17.5f

#define ACCEL_MODE 1 // Use 7 for normal mode, 1 for high accuracy mode.

// hello my name is william

esp_err_t reg_write(spi_device_handle_t dev, uint8_t reg_addr, uint8_t data) {
    spi_transaction_t transmit = {
        .cmd = 0, // 0 = write
        .addr = reg_addr,
        .length = 8, // data phase length: 8
        .flags = SPI_TRANS_USE_TXDATA,
        .tx_data = {data}
    };
    return spi_device_polling_transmit(dev, &transmit);
}

esp_err_t select_embedded_bank(spi_device_handle_t dev) {
    return reg_write(dev, FUNC_CFG_ACCESS, EMB_FUNC_REG_ACCESS);
}

esp_err_t select_main_bank(spi_device_handle_t dev) {
    return reg_write(dev, FUNC_CFG_ACCESS, 0x00);
}

esp_err_t reg_read(spi_device_handle_t dev, uint8_t reg_addr, uint8_t *data) {
    spi_transaction_t transmit = {
        .cmd = 1, // 1 = read
        .addr = reg_addr,
        .length = 8, // listens for 8 bits of data following command and address phases
        .flags = SPI_TRANS_USE_RXDATA,
    };
    esp_err_t err = spi_device_polling_transmit(dev, &transmit);
    *data = transmit.rx_data[0];
    return err;
}


void low_g_accel_gyro_config(spi_device_handle_t dev) {
    //Low g accel config
    uint8_t accel_control = (ACCEL_MODE << 4) | 6; // 0x76
    ESP_ERROR_CHECK(reg_write(dev, CTRL1, accel_control));
    //gyro config
    uint8_t gyro_dsp_control = (1 << 4) | 10; // 0x1A
    uint8_t gyro_control = (1 << 4) | 6; // 0x76
    ESP_ERROR_CHECK(reg_write(dev, CTRL6, gyro_dsp_control));
    ESP_ERROR_CHECK(reg_write(dev, CTRL2, gyro_control));

    // enable sflp in embedded functions
    ESP_ERROR_CHECK(select_embedded_bank(dev));
    ESP_ERROR_CHECK(reg_write(dev, EMB_FUNC_EN_A, SFLP_GAME_EN));
    ESP_ERROR_CHECK(select_main_bank(dev));
    
    return;
}

esp_err_t low_g_accel_read(spi_device_handle_t dev, uint8_t *data) {
    reg_read(dev, OUTX_L_A, &data[0]);
    reg_read(dev, OUTX_H_A, &data[1]);
    return ESP_OK;
}


int16_t read_imu(spi_device_handle_t dev, uint8_t low_reg)
{
    uint8_t low;
    uint8_t high;

    reg_read(dev, low_reg, &low);
    reg_read(dev, low_reg + 1, &high);

    int16_t data = (int16_t)((high << 8) | low);

    return data;
}

void app_main(void)
{   
    // INIT BUS
    spi_bus_config_t buscfg = {
    .mosi_io_num = MOSI,
    .miso_io_num = MISO,
    .sclk_io_num = CLK,
    .quadwp_io_num = -1,      // -1 = unused (only for quad SPI)
    .quadhd_io_num = -1,
    .max_transfer_sz = SPI_DMA_DISABLED,  // bytes; sizes the DMA descriptors
    };

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));


    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10 * 1000 * 1000,  // 10 MHz
        .mode = 0,                           // CPOL/CPHA: 0–3
        .spics_io_num = LSM_CS,                   // CS pin, or -1 to self drive
        .queue_size = 7,                     // max queued transactions
        .command_bits = 1,                   // optional built-in phases
        .address_bits = 7,
        .dummy_bits = 0                      // no dummy bits.
        // .flags = SPI_DEVICE_HALFDUPLEX,
        // .pre_cb = my_pre_cb,              // e.g. toggle a D/C line for displays
    };
    spi_device_handle_t dev;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &dev));

    uint8_t dev_id;
    reg_read(dev, 0x0F, &dev_id);
    printf("I am: %d!\n", dev_id);

    low_g_accel_gyro_config(dev);

    float pitch_deg = 0, roll_deg = 0, yaw_deg = 0;

    while (1) 
    {
        int16_t x = read_imu(dev, OUTX_L_A);
        int16_t y = read_imu(dev, OUTY_L_A);
        int16_t z = read_imu(dev, OUTZ_L_A);

        int16_t pitch = read_imu(dev, OUTX_L_G);
        int16_t roll = read_imu(dev, OUTY_L_G);
        int16_t yaw = read_imu(dev, OUTZ_L_G);

        // switch to embedded bank to read gravity vector
        select_embedded_bank(dev);
        int16_t gx_raw = read_imu(dev, SFLP_GRAVX_L);
        int16_t gy_raw = read_imu(dev, SFLP_GRAVY_L);
        int16_t gz_raw = read_imu(dev, SFLP_GRAVZ_L);
        select_main_bank(dev);

        float x_ms2 = x * ACCEL_SENS_MG_PER_LSB / 1000.0f * G_TO_MS2;
        float y_ms2 = y * ACCEL_SENS_MG_PER_LSB / 1000.0f * G_TO_MS2;
        float z_ms2 = z * ACCEL_SENS_MG_PER_LSB / 1000.0f * G_TO_MS2;

        

        float x_pitch = pitch * GYRO_SENS_MDPS_PER_LSB / 1000.0f;   // deg/s
        float y_roll  = roll  * GYRO_SENS_MDPS_PER_LSB / 1000.0f;
        float z_yaw   = yaw   * GYRO_SENS_MDPS_PER_LSB / 1000.0f;


        float gx_ms2 = gx_raw * ACCEL_SENS_MG_PER_LSB / 1000.0f * G_TO_MS2;
        float gy_ms2 = gy_raw * ACCEL_SENS_MG_PER_LSB / 1000.0f * G_TO_MS2;
        float gz_ms2 = gz_raw * ACCEL_SENS_MG_PER_LSB / 1000.0f * G_TO_MS2;

        float lin_x = x_ms2 - gx_ms2;
        float lin_y = y_ms2 - gy_ms2;
        float lin_z = z_ms2 - gz_ms2;

        //printf("X = %.2f, Y = %.2f, Z = %.2f m/s^2\n", x_ms2, y_ms2, z_ms2);
        pitch_deg += x_pitch * 0.1f;   // 0.1 s = your 100 ms delay
        roll_deg  += y_roll  * 0.1f;
        yaw_deg   += z_yaw   * 0.1f;

        printf("Lin Accel (m/s^2): X=%.2f Y=%.2f Z=%.2f | Angles: P=%.1f R=%.1f Y=%.1f\n", 
               lin_x, lin_y, lin_z, pitch_deg, roll_deg, yaw_deg);
        //printf("Pitch = %.1f, Roll = %.1f, Yaw = %.1f deg\n", pitch_deg, roll_deg, yaw_deg);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
