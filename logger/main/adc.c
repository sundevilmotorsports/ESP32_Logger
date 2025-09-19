#include "adc.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"



static uint8_t buffer[2];
static spi_device_handle_t spi_handle;

void adcInit(spi_device_handle_t spi) {
    buffer[0] = 0;
    buffer[1] = 0;
    spi_handle = spi;
}

void adc_enable(void){
    gpio_set_level(ADC_PIN, ADC_ENABLE);
    return;
}

void adc_disable(void){
    gpio_set_level(ADC_PIN, ADC_DISABLE);
    return;
}

uint16_t getAnalog(uint8_t channel) {
    spi_transaction_t t = {0};
    uint8_t channelInput = channel << 3;

    // First transaction: send channel, receive don't care
    t.length = 8;
    t.tx_buffer = &channelInput;
    t.rx_buffer = buffer;
    spi_device_transmit(spi_handle, &t);

    // Second transaction: send dummy, receive actual data
    uint8_t dummy = 0;
    t.tx_buffer = &dummy;
    t.rx_buffer = buffer;
    spi_device_transmit(spi_handle, &t);

    return (buffer[0] << 8) | buffer[1];
}