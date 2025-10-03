#ifndef CAN_H
#define CAN_H

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"

extern twai_node_handle_t hfdcan;

// Callback function type for message processing
typedef void (*can_message_callback_t)(twai_frame_t *message);


void can_init(can_message_callback_t callback_function);
#endif