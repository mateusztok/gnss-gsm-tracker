#include "gps_parser.h"
#include <stdio.h>
#include <string.h>

#define CGNSINF_PREFIX "+CGNSINF: "
#define CGNSINF_MAX_FIELDS 25U
#define CGNSINF_LINE_CAPACITY 256U

static void CopyField(char *destination, uint16_t capacity, const char *source)
{
    if (capacity == 0U) {
        return;
    }
    (void)snprintf(destination, capacity, "%s", source);
}

uint8_t GPS_Parser_ParseLine(const char *line, GPS_Data_t *data)
{
    char line_copy[CGNSINF_LINE_CAPACITY];
    char *fields[CGNSINF_MAX_FIELDS];
    char *cursor;
    uint16_t field_count = 1U;
    size_t line_length;

    if (line == NULL || data == NULL ||
        strncmp(line, CGNSINF_PREFIX, sizeof(CGNSINF_PREFIX) - 1U) != 0) {
        return 0U;
    }

    line += sizeof(CGNSINF_PREFIX) - 1U;
    line_length = strlen(line);
    if (line_length >= sizeof(line_copy)) {
        return 0U;
    }
    memcpy(line_copy, line, line_length + 1U);

    fields[0] = line_copy;
    for (cursor = line_copy; *cursor != '\0' && field_count < CGNSINF_MAX_FIELDS; ++cursor) {
        if (*cursor == ',') {
            *cursor = '\0';
            fields[field_count++] = cursor + 1;
        }
    }

    if (field_count < 7U) {
        return 0U;
    }

    if (strcmp(fields[1], "1") != 0) {
        data->has_fix = 0U;
        return 1U;
    }

    CopyField(data->date_time, sizeof(data->date_time), fields[2]);
    CopyField(data->latitude, sizeof(data->latitude), fields[3]);
    CopyField(data->longitude, sizeof(data->longitude), fields[4]);
    CopyField(data->altitude, sizeof(data->altitude), fields[5]);
    CopyField(data->speed, sizeof(data->speed),
              fields[6][0] == '\0' ? "0.0" : fields[6]);
    data->has_fix = 1U;
    return 1U;
}

uint8_t GPS_Parser_FormatTimestamp(const char *raw, char *formatted, uint16_t capacity)
{
    int written;

    if (raw == NULL || formatted == NULL || capacity < 22U || strlen(raw) < 14U) {
        return 0U;
    }

    written = snprintf(formatted, capacity, "%.4s-%.2s-%.2s%%20%.2s:%.2s:%.2s",
                       raw, raw + 4, raw + 6, raw + 8, raw + 10, raw + 12);
    return (written == 21 && (uint16_t)written < capacity) ? 1U : 0U;
}
