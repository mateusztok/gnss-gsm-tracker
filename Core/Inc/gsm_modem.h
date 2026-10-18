#ifndef GSM_MODEM_H
#define GSM_MODEM_H

#include "gps_parser.h"
#include "stm32l4xx_hal.h"

HAL_StatusTypeDef GSM_Modem_Init(UART_HandleTypeDef *modem_uart,
                                 UART_HandleTypeDef *debug_uart);
HAL_StatusTypeDef GSM_Modem_Configure(void);
HAL_StatusTypeDef GSM_Modem_RequestGps(void);
HAL_StatusTypeDef GSM_Modem_ProcessIncoming(GPS_Data_t *gps_data);
HAL_StatusTypeDef GSM_Modem_SendThingSpeak(const GPS_Data_t *gps_data,
                                           uint8_t is_driving);
void GSM_Modem_OnRxComplete(UART_HandleTypeDef *uart);
void GSM_Modem_OnError(UART_HandleTypeDef *uart);

#endif
