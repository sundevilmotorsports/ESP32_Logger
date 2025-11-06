/*
 * adc.h
 *
 *  Created on: Mar 18, 2024
 *      Author: joshl
 */
#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"


// #define ADC_ENABLE  1
// #define ADC_DISABLE 0
// #define ADC_FBP 0
// #define ADC_RBP 1
// #define ADC_STP 2
// #define ADC_FRS 3
// #define ADC_FLS 4
// #define ADC_CH5 5
// #define ADC_RLS 6
// #define ADC_RRS 7

// typedef struct {
//     uint16_t value;
//     esp_err_t error;
// } ADC_Result;

typedef struct {
    uint16_t adc0;
    uint16_t adc1;
    uint16_t adc2;
    uint16_t adc3;
    uint16_t adc4;
    uint16_t adc5;
    uint16_t adc6;
    uint16_t adc7;
} adc_values_t;

esp_err_t adc_init( void );
esp_err_t adc_read_sync(adc_values_t *out);


#endif /* ADC_H_ */