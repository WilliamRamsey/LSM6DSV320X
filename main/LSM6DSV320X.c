#include <stdio.h>
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

// PIN MAPPING
#define MOSI 23
#define MISO 19
#define LSM_CS 5
#define CLK 18

// IMU REGISTER ADDRESSES
# define CTRL1 0x10



/*
esp_err_t low_g_accel_read(dev, uint8_t *data) {

}
*/


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


esp_err_t low_g_accel_config(spi_device_handle_t dev) {
    uint8_t data = (7 << 4) | 6; // 0x76
    return reg_write(dev, CTRL1, data);
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

    ESP_ERROR_CHECK(low_g_accel_config(dev));
}
