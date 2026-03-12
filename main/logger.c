/*
 * logger.c
 *
 *  Created on: Mar 23, 2024
 *      Author: joshl
 *   
 *  Copied on: Sept 6, 2025
 *       Copier: Alex R
 */
#include "logger.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"

static const char *TAG = "LOGGER";

// Create a ring buffer
log_ring_t* log_ring_create(size_t log_size, size_t capacity) {
    log_ring_t* ring = (log_ring_t*)malloc(sizeof(log_ring_t));
    if (!ring) {
        ESP_LOGE(TAG, "Failed to allocate ring structure");
        return NULL;
    }

    ring->buffer = (uint8_t*)malloc(log_size * capacity);
    if (!ring->buffer) {
        ESP_LOGE(TAG, "Failed to allocate ring buffer");
        free(ring);
        return NULL;
    }

    ring->log_size = log_size;
    ring->capacity = capacity;
    ring->head = 0;
    ring->tail = 0;
    ring->count = 0;
    ring->overruns = 0;
    ring->mutex = xSemaphoreCreateMutex();

    if (!ring->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        free(ring->buffer);
        free(ring);
        return NULL;
    }

    ESP_LOGI(TAG, "Ring buffer created: %u entries x %u bytes = %u total bytes",
             capacity, log_size, capacity * log_size);
    return ring;
}

// Write one log entry (returns 0 on success, -1 if full)
int log_ring_write(log_ring_t* ring, const uint8_t* data) {
    if (xSemaphoreTake(ring->mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "Write mutex timeout");
        return -1;
    }

    if (ring->count >= ring->capacity) {
        ring->overruns++;
        xSemaphoreGive(ring->mutex);
        return -1;  // Buffer full
    }

    // Copy data into ring buffer at head position
    memcpy(ring->buffer + (ring->head * ring->log_size), data, ring->log_size);
    
    ring->head = (ring->head + 1) % ring->capacity;
    ring->count++;

    xSemaphoreGive(ring->mutex);
    return 0;
}

// Read one log entry (returns 0 on success, -1 if empty)
int log_ring_read(log_ring_t* ring, uint8_t* data) {
    if (xSemaphoreTake(ring->mutex, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "Read mutex timeout");
        return -1;
    }

    if (ring->count == 0) {
        xSemaphoreGive(ring->mutex);
        return -1;  // Buffer empty
    }

    // Copy data from ring buffer at tail position
    memcpy(data, ring->buffer + (ring->tail * ring->log_size), ring->log_size);
    
    ring->tail = (ring->tail + 1) % ring->capacity;
    ring->count--;

    xSemaphoreGive(ring->mutex);
    return 0;
}



void loggerEmplaceU16(uint8_t* buffer, size_t addr, uint16_t data) {
    // little-endian (LSB first)
    buffer[addr + 0] = (uint8_t)(data & 0xff);
    buffer[addr + 1] = (uint8_t)(data >> 8);
}

void loggerEmplaceU32(uint8_t* buffer, size_t addr, uint32_t data) {
    // little-endian (LSB first)
    buffer[addr + 0] = (uint8_t)(data & 0xff);
    buffer[addr + 1] = (uint8_t)((data >> 8) & 0xff);
    buffer[addr + 2] = (uint8_t)((data >> 16) & 0xff);
    buffer[addr + 3] = (uint8_t)((data >> 24) & 0xff);
}

void loggerEmplaceU64(uint8_t* buffer, size_t addr, uint64_t data) {
    // little-endian (LSB first) — easier to parse on x86 hosts with direct memcpy
    buffer[addr + 0] = (uint8_t)(data & 0xff);
    buffer[addr + 1] = (uint8_t)((data >> 8) & 0xff);
    buffer[addr + 2] = (uint8_t)((data >> 16) & 0xff);
    buffer[addr + 3] = (uint8_t)((data >> 24) & 0xff);
    buffer[addr + 4] = (uint8_t)((data >> 32) & 0xff);
    buffer[addr + 5] = (uint8_t)((data >> 40) & 0xff);
    buffer[addr + 6] = (uint8_t)((data >> 48) & 0xff);
    buffer[addr + 7] = (uint8_t)((data >> 56) & 0xff);
}