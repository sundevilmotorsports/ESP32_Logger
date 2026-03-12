/*
 * logger.h
 *
 *  Created on: Mar 23, 2024
 *      Author: joshl
 *  
 *  Copied on: Sept 6, 2025
 *       Copier: Alex R
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include <stdint.h>
#include <stddef.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"



// Ring buffer to store multiple log snapshots
typedef struct {
    uint8_t* buffer;           // Flat buffer: capacity * log_size bytes
    size_t log_size;           // Size of one log entry (e.g., CH_COUNT)
    size_t capacity;           // Number of log entries the ring can hold
    size_t head;               // Write position (producer)
    size_t tail;               // Read position (consumer)
    size_t count;              // Current number of entries in buffer
    SemaphoreHandle_t mutex;   // Protects head/tail/count
    uint32_t overruns;         // Counts dropped logs when full
} log_ring_t;

// Initialize ring buffer
log_ring_t* log_ring_create(size_t log_size, size_t capacity);

// Producer: Write one log entry (non-blocking)
int log_ring_write(log_ring_t* ring, const uint8_t* data);

// Consumer: Read one log entry (non-blocking)
int log_ring_read(log_ring_t* ring, uint8_t* data);

void loggerEmplaceU16(uint8_t* buffer, size_t addr, uint16_t data);
void loggerEmplaceU32(uint8_t* buffer, size_t addr, uint32_t data);
void loggerEmplaceU64(uint8_t* buffer, size_t addr, uint64_t data);
void loggerEmplaceCAN(uint8_t* buffer, size_t addr, uint8_t* msg);
#endif /* INC_LOGGER_H_ */
