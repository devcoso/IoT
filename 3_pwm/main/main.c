#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/gpio.h" // Se incluye para el control de los pines de dirección
#include "esp_log.h"
#include "sdkconfig.h"

// Define los GPIO para el control del puente H
#define MOTOR_PIN_PWM (2) // Pin para el control de velocidad (Enable)
#define MOTOR_PIN_IN1 (4) // Pin para la dirección 1
#define MOTOR_PIN_IN2 (15) // Pin para la dirección 2

// Parámetros PWM
#define LEDC_TIMER_0 LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define PWM_FREQ_HZ (5000)
#define PWM_RESOLUTION (LEDC_TIMER_10_BIT)

// Canal PWM
#define LEDC_CHANNEL_SPEED LEDC_CHANNEL_0

static const char *TAG = "MOTOR_CONTROL";

void setup_pwm() {
    // Configuración del temporizador PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configuración del canal PWM para el control de velocidad (GPIO 2)
    ledc_channel_config_t ledc_channel_speed = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_SPEED,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_PIN_PWM,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_speed));

    ESP_LOGI(TAG, "PWM configurado en el GPIO %d.", MOTOR_PIN_PWM);
}

void setup_gpio_hbridge() {
    // Configura los pines de dirección como salida
    gpio_reset_pin(MOTOR_PIN_IN1);
    gpio_reset_pin(MOTOR_PIN_IN2);
    gpio_set_direction(MOTOR_PIN_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_PIN_IN2, GPIO_MODE_OUTPUT);

    // Asegura que el motor esté detenido al inicio
    gpio_set_level(MOTOR_PIN_IN1, 0);
    gpio_set_level(MOTOR_PIN_IN2, 0);

    ESP_LOGI(TAG, "Pines de dirección del puente H configurados en los GPIOs %d y %d.", MOTOR_PIN_IN1, MOTOR_PIN_IN2);
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

void set_motor_speed(int duty_percent) {
    int max_duty = (1 << PWM_RESOLUTION) - 1;
    int new_duty_value = 0;
    
    // Si el valor es positivo, mover hacia adelante
    if (duty_percent > 0) {
        new_duty_value = (int)((float)duty_percent / 100.0 * max_duty);
        gpio_set_level(MOTOR_PIN_IN1, 1);
        gpio_set_level(MOTOR_PIN_IN2, 0);
        ESP_LOGI(TAG, "Motor adelante: %d%%. Ciclo de trabajo aplicado: %d", duty_percent, new_duty_value);
    // Si el valor es negativo, mover en reversa
    } else if (duty_percent < 0) {
        new_duty_value = (int)((float)abs(duty_percent) / 100.0 * max_duty);
        gpio_set_level(MOTOR_PIN_IN1, 0);
        gpio_set_level(MOTOR_PIN_IN2, 1);
        ESP_LOGI(TAG, "Motor reversa: %d%%. Ciclo de trabajo aplicado: %d", abs(duty_percent), new_duty_value);
    // Si es 0, detener el motor
    } else {
        gpio_set_level(MOTOR_PIN_IN1, 0);
        gpio_set_level(MOTOR_PIN_IN2, 0);
        new_duty_value = 0;
        ESP_LOGI(TAG, "Motor detenido.");
    }
    
    // Aplicar el nuevo ciclo de trabajo al canal PWM
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_SPEED, new_duty_value);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_SPEED);
}

void app_main(void)
{
    setup_pwm();
    setup_gpio_hbridge(); // Nueva función para configurar los GPIOs del puente H
    setup_uart();

    uint8_t rx_buffer[32];
    int rx_index = 0;
    int duty_cycle_percent;

    while (1) {
        int rx_bytes = uart_read_bytes(UART_NUM_0, rx_buffer + rx_index, 1, pdMS_TO_TICKS(10));

        if (rx_bytes > 0) {
            char received_char = rx_buffer[rx_index];

            if (received_char == '\n' || received_char == '\r') {
                rx_buffer[rx_index] = '\0'; // Terminador de cadena
                
                if (sscanf((const char *)rx_buffer, "%d", &duty_cycle_percent) == 1) {
                    if (duty_cycle_percent >= -100 && duty_cycle_percent <= 100) {
                        set_motor_speed(duty_cycle_percent);
                    } else {
                        ESP_LOGW(TAG, "Valor fuera de rango (-100 a 100).");
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