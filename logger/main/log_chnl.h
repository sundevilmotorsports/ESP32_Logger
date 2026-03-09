#ifndef LOG_CHANNELS_H
#define LOG_CHANNELS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Defines one packed field per logical signal. Offsets below are generated
// from this layout, so adding/removing fields only requires editing log_fields.def.
typedef struct __attribute__((packed)) {
#define X(field_name, field_size) uint8_t field_name[(field_size)];
#include "log_fields.def"
#undef X
} log_frame_layout_t;

enum LogChannel {
#define X(field_name, field_size) field_name = offsetof(log_frame_layout_t, field_name),
#include "log_fields.def"
#undef X
    CH_COUNT = sizeof(log_frame_layout_t)
};

// Helper to address continuation bytes without defining *_1, *_2, *_3 symbols.
#define LOG_BYTE(field, index) ((field) + (index))

typedef struct {
    const char *name;
    uint16_t offset;
    uint8_t size;
} log_field_desc_t;

size_t log_field_desc_count(void);
const log_field_desc_t *log_field_desc_at(size_t index);
size_t log_csv_header_required_len(bool include_ch_count_label);
size_t log_build_csv_header(char *out, size_t out_cap, bool include_ch_count_label);

extern uint8_t logBuffer[CH_COUNT];

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
} imu_accel_t;

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;
} imu_gyro_t;

extern imu_accel_t imu_accel;
extern imu_gyro_t  imu_gyro;
extern uint8_t ect, tps, aps, shift0, shift1, shift2;

#endif // LOG_CHANNELS_H