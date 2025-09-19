/*
 * adc.h
 *
 *  Created on: Mar 18, 2024
 *      Author: joshl
 */
#ifndef INC_ADC_H_
#define INC_ADC_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"


#define ADC_PIN GPIO_NUM_4
#define ADC_ENABLE  1
#define ADC_DISABLE 0

#define ADC_FBP 0
#define ADC_RBP 1
#define ADC_STP 2
#define ADC_FRS 3
#define ADC_FLS 4
#define ADC_CH5 5
#define ADC_RLS 6
#define ADC_RRS 7

void adcInit();
void adc_enable();
void adc_disable();

typedef struct {
    uint16_t value;
    esp_err_t error;
} ADC_Result;

uint16_t getAnalog(uint8_t channel);


#endif /* INC_ADC_H_ */