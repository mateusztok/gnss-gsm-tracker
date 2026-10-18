#ifndef GPS_PARSER_H
#define GPS_PARSER_H

#include <stdint.h>

typedef struct {
    char date_time[32];
    char latitude[16];
    char longitude[16];
    char altitude[10];
    char speed[10];
    uint8_t has_fix;
} GPS_Data_t;

uint8_t GPS_Parser_ParseLine(const char *line, GPS_Data_t *data);
uint8_t GPS_Parser_FormatTimestamp(const char *raw, char *formatted, uint16_t capacity);

#endif
