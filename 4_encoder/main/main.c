#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_log.h"
#include "sdkconfig.h"

// Define los GPIO para el control del puente H
#define MOTOR_PIN_IN1 (2)
#define MOTOR_PIN_IN2 (4)
#define MOTOR_PIN_PWM_ENA (5) // Pin para habilitación y control PWM

// Parámetros PWM
#define LEDC_TIMER_0 LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define PWM_FREQ_HZ (5000)
#define PWM_RESOLUTION (LEDC_TIMER_10_BIT)

// Canales PWM
#define LEDC_CHANNEL_ENA LEDC_CHANNEL_0

// Define los pines del encoder
#define ENCODER_PCNT_UNIT PCNT_UNIT_0
#define ENCODER_PIN_A (25)
#define ENCODER_PIN_B (26)
#define COUNTS_PER_OUTPUT_REVOLUTION (1496.0f)

static const char *TAG = "MOTOR_ENCODER";

// Variable para el contador del encoder y el duty cycle, accesibles por ambas tareas
volatile int16_t encoder_ticks = 0;
volatile int motor_duty_percent = 0;

// Define la estructura de datos que se enviará
typedef struct {
    uint8_t start_byte;
    int8_t motor_direction; // Nuevo campo para la dirección
    float motor_turns;
    uint8_t end_byte;
} __attribute__((packed)) MotorDataPacket_t;

void setup_pwm() {
    // Configura los pines IN1 e IN2 como salidas digitales
    gpio_set_direction(MOTOR_PIN_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_PIN_IN2, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_PIN_IN1, 0);
    gpio_set_level(MOTOR_PIN_IN2, 0);

    // Configura el temporizador PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configura el canal PWM para el pin ENA
    ledc_channel_config_t ledc_channel_ena = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_ENA,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_PIN_PWM_ENA,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_ena));

    ESP_LOGI(TAG, "PWM configurado en el GPIO %d (ENA).", MOTOR_PIN_PWM_ENA);
    ESP_LOGI(TAG, "Pines de control (IN1, IN2) en los GPIOs %d y %d.", MOTOR_PIN_IN1, MOTOR_PIN_IN2);
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
    
    if (duty_percent > 0) {
        new_duty_value = (int)((float)duty_percent / 100.0 * max_duty);
        gpio_set_level(MOTOR_PIN_IN1, 1); // Adelante
        gpio_set_level(MOTOR_PIN_IN2, 0);
        ESP_LOGI(TAG, "Motor adelante: %d%%. Ciclo de trabajo: %d", duty_percent, new_duty_value);
    } else if (duty_percent < 0) {
        new_duty_value = (int)((float)abs(duty_percent) / 100.0 * max_duty);
        gpio_set_level(MOTOR_PIN_IN1, 0);
        gpio_set_level(MOTOR_PIN_IN2, 1); // Reversa
        ESP_LOGI(TAG, "Motor reversa: %d%%. Ciclo de trabajo: %d", abs(duty_percent), new_duty_value);
    } else {
        gpio_set_level(MOTOR_PIN_IN1, 0); // Freno
        gpio_set_level(MOTOR_PIN_IN2, 0);
        new_duty_value = 0;
        ESP_LOGI(TAG, "Motor detenido.");
    }
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_ENA, new_duty_value);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_ENA);
}

void setup_encoder() {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = ENCODER_PIN_A,
        .ctrl_gpio_num = ENCODER_PIN_B,
        .channel = PCNT_CHANNEL_0,
        .unit = ENCODER_PCNT_UNIT,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DEC,
        .lctrl_mode = PCNT_MODE_REVERSE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = 0,
        .counter_l_lim = 0,
    };
    pcnt_unit_config(&pcnt_config);
    
    pcnt_config.pulse_gpio_num = ENCODER_PIN_B;
    pcnt_config.ctrl_gpio_num = ENCODER_PIN_A;
    pcnt_config.channel = PCNT_CHANNEL_1;
    pcnt_config.pos_mode = PCNT_COUNT_INC,
    pcnt_config.neg_mode = PCNT_COUNT_DEC,
    pcnt_unit_config(&pcnt_config);

    pcnt_set_filter_value(ENCODER_PCNT_UNIT, 100);
    pcnt_filter_enable(ENCODER_PCNT_UNIT);

    pcnt_counter_pause(ENCODER_PCNT_UNIT);
    pcnt_counter_clear(ENCODER_PCNT_UNIT);
    pcnt_counter_resume(ENCODER_PCNT_UNIT);

    ESP_LOGI(TAG, "Encoder configurado en la unidad PCNT %d.", ENCODER_PCNT_UNIT);
}

void encoder_task(void *pvParameters) {
    MotorDataPacket_t data_packet;
    data_packet.start_byte = 0xA5;
    data_packet.end_byte = 0x5A;

    while(1) {
        pcnt_get_counter_value(ENCODER_PCNT_UNIT, &encoder_ticks);
        
        // Calcula el número de vueltas
        float motor_turns = (float)encoder_ticks / COUNTS_PER_OUTPUT_REVOLUTION;

        // Determina la dirección del motor
        if (motor_duty_percent > 0) {
            data_packet.motor_direction = 1; // Adelante
        } else if (motor_duty_percent < 0) {
            data_packet.motor_direction = -1; // Reversa
        } else {
            data_packet.motor_direction = 0; // Detenido
        }
        
        data_packet.motor_turns = motor_turns;
        
        uart_write_bytes(UART_NUM_0, (const char *)&data_packet, sizeof(MotorDataPacket_t));
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void motor_control_task(void *pvParameters) {
    uint8_t rx_buffer[32];
    int rx_index = 0;
    
    while (1) {
        int rx_bytes = uart_read_bytes(UART_NUM_0, rx_buffer + rx_index, 1, pdMS_TO_TICKS(10));
        if (rx_bytes > 0) {
            char received_char = rx_buffer[rx_index];
            if (received_char == '\n' || received_char == '\r') {
                rx_buffer[rx_index] = '\0';
                if (sscanf((const char *)rx_buffer, "%d", (int*)&motor_duty_percent) == 1) {
                    if (motor_duty_percent >= -100 && motor_duty_percent <= 100) {
                        set_motor_speed(motor_duty_percent);
                    } else {
                        ESP_LOGW(TAG, "Valor fuera de rango (-100 a 100).");
                    }
                } else {
                    ESP_LOGW(TAG, "No se pudo convertir a entero.");
                }
                rx_index = 0;
            } else {
                rx_index++;
                if (rx_index >= sizeof(rx_buffer) - 1) {
                    rx_index = 0;
                    ESP_LOGW(TAG, "Buffer de recepcion desbordado.");
                }
            }
        }
    }
}

void app_main(void) {
    setup_pwm();
    setup_uart();
    setup_encoder();
    
    xTaskCreate(motor_control_task, "motor_control", 2048, NULL, 5, NULL);
    xTaskCreate(encoder_task, "encoder_read", 2048, NULL, 5, NULL);
}