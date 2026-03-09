#include "log_chnl.h"

#include <stdio.h>
#include <string.h>

typedef struct {
    const char *name;
    uint8_t size;
} log_field_raw_t;

static const log_field_raw_t k_fields[] = {
#define X(field_name, field_size) {#field_name, (uint8_t)(field_size)},
#include "log_fields.def"
#undef X
};

static const uint16_t k_offsets[] = {
#define X(field_name, field_size) (uint16_t)field_name,
#include "log_fields.def"
#undef X
};

size_t log_field_desc_count(void)
{
    return sizeof(k_fields) / sizeof(k_fields[0]);
}

const log_field_desc_t *log_field_desc_at(size_t index)
{
    static log_field_desc_t desc;

    if (index >= log_field_desc_count()) {
        return NULL;
    }

    desc.name = k_fields[index].name;
    desc.offset = k_offsets[index];
    desc.size = k_fields[index].size;
    return &desc;
}

static size_t append_label(char *out, size_t out_cap, size_t pos, const char *name, uint32_t suffix)
{
    char token[40];
    int n;
    size_t token_len;

    if (suffix == 0U) {
        n = snprintf(token, sizeof(token), "%s,", name);
    } else {
        n = snprintf(token, sizeof(token), "%s%u,", name, (unsigned)suffix);
    }

    if (n < 0) {
        return pos;
    }

    token_len = (size_t)n;

    if (out != NULL && out_cap > 0U && pos < out_cap - 1U) {
        size_t room = (out_cap - 1U) - pos;
        size_t copy_len = token_len < room ? token_len : room;
        memcpy(out + pos, token, copy_len);
    }

    return pos + token_len;
}

size_t log_csv_header_required_len(bool include_ch_count_label)
{
    return log_build_csv_header(NULL, 0U, include_ch_count_label);
}

size_t log_build_csv_header(char *out, size_t out_cap, bool include_ch_count_label)
{
    size_t pos = 0;
    size_t field_count = log_field_desc_count();
    size_t i;

    for (i = 0; i < field_count; i++) {
        uint8_t byte_index;
        const log_field_desc_t *desc = log_field_desc_at(i);
        if (desc == NULL) {
            continue;
        }

        for (byte_index = 0; byte_index < desc->size; byte_index++) {
            pos = append_label(out, out_cap, pos, desc->name, byte_index);
        }
    }

    if (include_ch_count_label) {
        pos = append_label(out, out_cap, pos, "CH_COUNT", 0U);
    }

    if (out != NULL && out_cap > 0U) {
        size_t term = pos < (out_cap - 1U) ? pos : (out_cap - 1U);
        out[term] = '\0';
    }

    return pos;
}
