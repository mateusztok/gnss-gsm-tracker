#include "main.h"
#include "config.h"
#include "gps_parser.h"
#include "gsm_modem.h"
#include "mpu6050.h"
#include <string.h>

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;

typedef enum {
    TRACKER_MODE_DRIVING,
    TRACKER_MODE_PARKING,
    TRACKER_MODE_SLEEP
} TrackerMode_t;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void UpdateStatusLed(TrackerMode_t mode, uint32_t now);
static void DebugPrint(const char *message);

int main(void)
{
    GPS_Data_t gps_data = {0};
    MPU6050_t mpu = {0};
    TrackerMode_t mode = TRACKER_MODE_DRIVING;
    uint32_t last_movement_time;
    uint32_t motion_check_time;
    uint32_t gps_poll_time;
    uint32_t upload_time;

    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_LPUART1_UART_Init();
    MX_I2C1_Init();

    if (MPU6050_Init(&mpu, &hi2c1) != HAL_OK) {
        Error_Handler();
    }

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(5000);

    if (GSM_Modem_Init(&hlpuart1, &huart2) != HAL_OK ||
        GSM_Modem_Configure() != HAL_OK) {
        Error_Handler();
    }

    last_movement_time = HAL_GetTick();
    motion_check_time = last_movement_time;
    gps_poll_time = last_movement_time;
    upload_time = last_movement_time;
    DebugPrint("[INFO] Tracker uruchomiony\r\n");

    while (1) {
        uint32_t now = HAL_GetTick();
        uint32_t idle_time;
        uint8_t movement_detected = 0;

        if (GSM_Modem_ProcessIncoming(&gps_data) != HAL_OK) {
            Error_Handler();
        }

        if ((uint32_t)(now - motion_check_time) >= MOTION_CHECK_RATE_MS) {
            if (MPU6050_DetectMotion(&mpu, MOTION_THRESHOLD, &movement_detected) != HAL_OK) {
                Error_Handler();
            }

            motion_check_time = now;
            if (movement_detected != 0U) {
                last_movement_time = now;
                if (mode == TRACKER_MODE_SLEEP) {
                    DebugPrint("[ALARM] Wykryto ruch. Wybudzanie...\r\n");
                    upload_time = 0;
                }
            }
        }

        idle_time = (uint32_t)(now - last_movement_time);
        if (idle_time < TIME_TO_PARKING_MS) {
            mode = TRACKER_MODE_DRIVING;
        } else if (idle_time < TIME_TO_SLEEP_MS) {
            mode = TRACKER_MODE_PARKING;
        } else {
            mode = TRACKER_MODE_SLEEP;
        }

        switch (mode) {
        case TRACKER_MODE_DRIVING:
            if ((uint32_t)(now - gps_poll_time) >= GPS_POLL_DRIVING_MS) {
                if (GSM_Modem_RequestGps() != HAL_OK) {
                    Error_Handler();
                }
                gps_poll_time = now;
            }

            if ((uint32_t)(now - upload_time) >= UPLOAD_DRIVING_MS) {
                if (gps_data.has_fix != 0U &&
                    GSM_Modem_SendThingSpeak(&gps_data, 1U) != HAL_OK) {
                    Error_Handler();
                }
                upload_time = now;
            }
            break;

        case TRACKER_MODE_PARKING:
            if ((uint32_t)(now - upload_time) >= UPLOAD_PARKING_MS) {
                if (GSM_Modem_RequestGps() != HAL_OK) {
                    Error_Handler();
                }
                HAL_Delay(GPS_PARKING_RESPONSE_WAIT_MS);
                if (GSM_Modem_ProcessIncoming(&gps_data) != HAL_OK) {
                    Error_Handler();
                }
                if (gps_data.has_fix != 0U &&
                    GSM_Modem_SendThingSpeak(&gps_data, 0U) != HAL_OK) {
                    Error_Handler();
                }
                upload_time = HAL_GetTick();
            }
            break;

        case TRACKER_MODE_SLEEP:
        default:
            break;
        }

        UpdateStatusLed(mode, HAL_GetTick());
    }
}

static void DebugPrint(const char *message)
{
    if (HAL_UART_Transmit(&huart2, (uint8_t *)message, (uint16_t)strlen(message), 100U) != HAL_OK) {
        Error_Handler();
    }
}

static void UpdateStatusLed(TrackerMode_t mode, uint32_t now)
{
    static uint32_t last_toggle_time = 0;
    uint32_t interval;

    switch (mode) {
    case TRACKER_MODE_DRIVING:
        interval = LED_DRIVING_INTERVAL_MS;
        break;
    case TRACKER_MODE_PARKING:
        interval = LED_PARKING_INTERVAL_MS;
        break;
    case TRACKER_MODE_SLEEP:
    default:
        interval = LED_SLEEP_INTERVAL_MS;
        break;
    }

    if ((uint32_t)(now - last_toggle_time) >= interval) {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        last_toggle_time = now;
    }
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef oscillator = {0};
    RCC_ClkInitTypeDef clock = {0};

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        Error_Handler();
    }

    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    oscillator.HSIState = RCC_HSI_ON;
    oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    oscillator.PLL.PLLM = 1;
    oscillator.PLL.PLLN = 10;
    oscillator.PLL.PLLP = RCC_PLLP_DIV7;
    oscillator.PLL.PLLQ = RCC_PLLQ_DIV2;
    oscillator.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        Error_Handler();
    }

    clock.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV1;
    clock.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_4) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x00707CBB;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK ||
        HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK ||
        HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_LPUART1_UART_Init(void)
{
    hlpuart1.Instance = LPUART1;
    hlpuart1.Init.BaudRate = 115200;
    hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits = UART_STOPBITS_1;
    hlpuart1.Init.Parity = UART_PARITY_NONE;
    hlpuart1.Init.Mode = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&hlpuart1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);

    gpio.Pin = B1_Pin;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &gpio);

    gpio.Pin = LD2_Pin;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &gpio);

    gpio.Pin = GPIO_PIN_6;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    GSM_Modem_OnRxComplete(uart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    GSM_Modem_OnError(uart);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}
