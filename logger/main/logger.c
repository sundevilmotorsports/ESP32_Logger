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