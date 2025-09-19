#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "sdkconfig.h"

// Define el GPIO para la salida PWM
#define LEDC_OUTPUT_IO (2)
#define LEDC_CHANNEL_0 LEDC_CHANNEL_0
#define LEDC_TIMER_0 LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE

// Parámetros de la onda PWM
#define PWM_FREQ_HZ (5000)
#define PWM_RESOLUTION (LEDC_TIMER_10_BIT)

static const char *TAG = "PWM_CONTROL";

void setup_pwm() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ESP_LOGI(TAG, "PWM configurado en el GPIO %d.", LEDC_OUTPUT_IO);
}

void setup_uart() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(UART_NUM_0, 256 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
    ESP_LOGI(TAG, "UART0 configurado.");
}

void app_main(void)
{
    setup_pwm();
    setup_uart();

    uint8_t rx_buffer[32];
    int rx_index = 0;
    int duty_cycle_percent;
    int max_duty = (1 << PWM_RESOLUTION) - 1;

    while (1) {
        int rx_bytes = uart_read_bytes(UART_NUM_0, rx_buffer + rx_index, 1, pdMS_TO_TICKS(10));

        if (rx_bytes > 0) {
            char received_char = rx_buffer[rx_index];

            if (received_char == '\n' || received_char == '\r') {
                rx_buffer[rx_index] = '\0'; // Terminador de cadena
                
                if (sscanf((const char *)rx_buffer, "%d", &duty_cycle_percent) == 1) {
                    if (duty_cycle_percent >= 0 && duty_cycle_percent <= 100) {
                        int new_duty = (int)((float)duty_cycle_percent / 100.0 * max_duty);
                        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, new_duty);
                        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
                        ESP_LOGI(TAG, "Valor recibido: %d%%. Ciclo de trabajo aplicado: %d", duty_cycle_percent, new_duty);
                    } else {
                        ESP_LOGW(TAG, "Valor fuera de rango (0-100).");
                    }
                } else {
                    ESP_LOGW(TAG, "No se pudo convertir a entero.");
                }
                // Reinicia el buffer
                rx_index = 0;
            } else {
                // Si no es un Enter, guarda el caracter y avanza el indice
                rx_index++;
                if (rx_index >= sizeof(rx_buffer) - 1) {
                    // Buffer lleno, reinicia para evitar desbordamiento
                    rx_index = 0;
                    ESP_LOGW(TAG, "Buffer de recepcion desbordado.");
                }
            }
        }
    }
}