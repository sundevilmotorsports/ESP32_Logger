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


#define ADC_CS GPIO_NUM_4
#define ADC_ENABLE  1
#define ADC_DISABLE 0
#define ADC_DIN GPIO_NUM_7
#define ADC_DOUT GPIO_NUM_6
#define ADC_CLK GPIO_NUM_5


#define ADC_FBP 0
#define ADC_RBP 1
#define ADC_STP 2
#define ADC_FRS 3
#define ADC_FLS 4
#define ADC_CH5 5
#define ADC_RLS 6
#define ADC_RRS 7



typedef struct {
    uint16_t value;
    esp_err_t error;
} ADC_Result;


esp_err_t adc_init(void);

uint16_t adc_get_channel(uint8_t channel);

esp_err_t adc_get_values(uint16_t *fbp, uint16_t *rbp, uint16_t *stp, 
                        uint16_t *fls, uint16_t *frs, uint16_t *rrs, uint16_t *rls);


#endif /* INC_ADC_H_ */