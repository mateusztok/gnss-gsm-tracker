#include "gsm_modem.h"
#include "config.h"
#include <stdio.h>
#include <string.h>

#define MODEM_RX_BUFFER_SIZE 1024U
#define MODEM_LINE_BUFFER_SIZE 256U

static UART_HandleTypeDef *modem_uart;
static UART_HandleTypeDef *debug_uart;
static uint8_t rx_byte;
static uint8_t rx_buffer[MODEM_RX_BUFFER_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint8_t rx_error;
static char line_buffer[MODEM_LINE_BUFFER_SIZE];
static uint16_t line_length;
static uint8_t discard_line;

static HAL_StatusTypeDef Transmit(const char *data, uint16_t length, uint32_t timeout)
{
    if (modem_uart == NULL || data == NULL) {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit(modem_uart, (uint8_t *)data, length, timeout);
}

static HAL_StatusTypeDef SendCommand(const char *command, uint32_t delay_ms)
{
    HAL_StatusTypeDef status = Transmit(command, (uint16_t)strlen(command), 1000U);
    if (status == HAL_OK && delay_ms != 0U) {
        HAL_Delay(delay_ms);
    }
    return status;
}

static HAL_StatusTypeDef DebugTransmit(const char *message, uint16_t length)
{
    if (debug_uart == NULL) {
        return HAL_ERROR;
    }
    return HAL_UART_Transmit(debug_uart, (uint8_t *)message, length, 100U);
}

static HAL_StatusTypeDef LogGpsFix(const GPS_Data_t *gps_data)
{
    char message[128];
    int length = snprintf(message, sizeof(message),
                          "[GPS] FIX: %s, %s (alt: %s m, speed: %s km/h)\r\n",
                          gps_data->latitude, gps_data->longitude,
                          gps_data->altitude, gps_data->speed);
    if (length < 0 || (size_t)length >= sizeof(message)) {
        return HAL_ERROR;
    }
    return DebugTransmit(message, (uint16_t)length);
}

HAL_StatusTypeDef GSM_Modem_Init(UART_HandleTypeDef *uart,
                                 UART_HandleTypeDef *console_uart)
{
    if (uart == NULL || console_uart == NULL) {
        return HAL_ERROR;
    }

    modem_uart = uart;
    debug_uart = console_uart;
    rx_head = 0U;
    rx_tail = 0U;
    rx_error = 0U;
    line_length = 0U;
    discard_line = 0U;

    return HAL_UART_Receive_IT(modem_uart, &rx_byte, 1U);
}

HAL_StatusTypeDef GSM_Modem_Configure(void)
{
    char apn_command[48];
    int length;

    if (SendCommand("AT\r\n", 500U) != HAL_OK ||
        SendCommand("ATE0\r\n", 500U) != HAL_OK ||
        SendCommand("AT+CGNSPWR=1\r\n", 1000U) != HAL_OK ||
        SendCommand("AT+CGATT=1\r\n", 2000U) != HAL_OK) {
        return HAL_ERROR;
    }

    length = snprintf(apn_command, sizeof(apn_command),
                      "AT+CSTT=\"%s\"\r\n", GSM_ACCESS_POINT_NAME);
    if (length < 0 || (size_t)length >= sizeof(apn_command)) {
        return HAL_ERROR;
    }
    if (SendCommand(apn_command, 1000U) != HAL_OK ||
        SendCommand("AT+CIICR\r\n", 3000U) != HAL_OK ||
        SendCommand("AT+CIFSR\r\n", 1000U) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef GSM_Modem_RequestGps(void)
{
    return SendCommand("AT+CGNSINF\r\n", 0U);
}

HAL_StatusTypeDef GSM_Modem_ProcessIncoming(GPS_Data_t *gps_data)
{
    if (gps_data == NULL || modem_uart == NULL || debug_uart == NULL || rx_error != 0U) {
        return HAL_ERROR;
    }

    while (rx_tail != rx_head) {
        uint8_t byte = rx_buffer[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) % MODEM_RX_BUFFER_SIZE);

        if (byte == '\r' || byte == '\n') {
            if (line_length != 0U && discard_line == 0U) {
                uint8_t parsed = GPS_Parser_ParseLine(line_buffer, gps_data);
                if (parsed != 0U && gps_data->has_fix != 0U &&
                    LogGpsFix(gps_data) != HAL_OK) {
                    return HAL_ERROR;
                }
            }
            line_length = 0U;
            discard_line = 0U;
        } else if (byte >= 32U && byte <= 126U && discard_line == 0U) {
            if (line_length < (MODEM_LINE_BUFFER_SIZE - 1U)) {
                line_buffer[line_length++] = (char)byte;
                line_buffer[line_length] = '\0';
            } else {
                discard_line = 1U;
            }
        }
    }

    return (rx_error == 0U) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef GSM_Modem_SendThingSpeak(const GPS_Data_t *gps_data,
                                           uint8_t is_driving)
{
    static const char start_connection[] =
        "AT+CIPSTART=\"TCP\",\"api.thingspeak.com\",\"80\"\r\n";
    static const char start_send[] = "AT+CIPSEND\r\n";
    char timestamp[24];
    char request[384];
    int request_length;
    uint8_t ctrl_z = 0x1AU;

    if (gps_data == NULL || gps_data->has_fix == 0U ||
        modem_uart == NULL || debug_uart == NULL) {
        return HAL_ERROR;
    }

    if (Transmit(start_connection, sizeof(start_connection) - 1U, 1000U) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(2000U);

    if (Transmit(start_send, sizeof(start_send) - 1U, 1000U) != HAL_OK) {
        return HAL_ERROR;
    }
    HAL_Delay(500U);

    if (GPS_Parser_FormatTimestamp(gps_data->date_time, timestamp, sizeof(timestamp)) != 0U) {
        request_length = snprintf(
            request, sizeof(request),
            "GET /update?api_key=%s&field1=%s&field2=%s&field3=%s&field5=%u&created_at=%s HTTP/1.1\r\n"
            "Host: api.thingspeak.com\r\n"
            "Connection: close\r\n\r\n",
            THINGSPEAK_API_KEY, gps_data->latitude, gps_data->longitude,
            gps_data->speed, (unsigned int)(is_driving != 0U), timestamp);
    } else {
        request_length = snprintf(
            request, sizeof(request),
            "GET /update?api_key=%s&field1=%s&field2=%s&field3=%s&field5=%u HTTP/1.1\r\n"
            "Host: api.thingspeak.com\r\n"
            "Connection: close\r\n\r\n",
            THINGSPEAK_API_KEY, gps_data->latitude, gps_data->longitude,
            gps_data->speed, (unsigned int)(is_driving != 0U));
    }

    if (request_length < 0 || (size_t)request_length >= sizeof(request)) {
        return HAL_ERROR;
    }
    if (Transmit(request, (uint16_t)request_length, 2000U) != HAL_OK ||
        HAL_UART_Transmit(modem_uart, &ctrl_z, 1U, 1000U) != HAL_OK) {
        return HAL_ERROR;
    }

    {
        char message[96];
        int message_length = snprintf(message, sizeof(message),
                                      "[INFO] Wyslano dane (tryb: %s)\r\n",
                                      is_driving != 0U ? "jazda" : "parking");
        if (message_length < 0 || (size_t)message_length >= sizeof(message)) {
            return HAL_ERROR;
        }
        return DebugTransmit(message, (uint16_t)message_length);
    }
}

void GSM_Modem_OnRxComplete(UART_HandleTypeDef *uart)
{
    uint16_t next_head;

    if (uart == NULL || uart != modem_uart) {
        return;
    }

    next_head = (uint16_t)((rx_head + 1U) % MODEM_RX_BUFFER_SIZE);
    if (next_head == rx_tail) {
        rx_error = 1U;
    } else {
        rx_buffer[rx_head] = rx_byte;
        rx_head = next_head;
    }

    if (HAL_UART_Receive_IT(modem_uart, &rx_byte, 1U) != HAL_OK) {
        rx_error = 1U;
    }
}

void GSM_Modem_OnError(UART_HandleTypeDef *uart)
{
    if (uart == NULL || uart != modem_uart) {
        return;
    }

    rx_error = 1U;
    if (HAL_UART_Receive_IT(modem_uart, &rx_byte, 1U) != HAL_OK) {
        rx_error = 1U;
    }
}
