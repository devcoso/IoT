#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h" // Driver moderno
#include "esp_log.h" 

// --- Pines ---
#define MOTOR_PIN_IN1       (2)
#define MOTOR_PIN_IN2       (4)
#define MOTOR_PIN_PWM_ENA   (5)

#define ENCODER_PIN_A       (25)
#define ENCODER_PIN_B       (26)

// --- PWM Config ---
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define PWM_FREQ_HZ         (5000)
#define PWM_RESOLUTION      (LEDC_TIMER_10_BIT)
#define LEDC_CHANNEL_ENA    LEDC_CHANNEL_0
#define MIN_PWM_DUTY        (40)     // 40% duty cycle

// --- Variables Globales y Manejadores ---
pcnt_unit_handle_t pcnt_unit = NULL;
volatile int encoder_ticks = 0; 
volatile float proportional_gain = 0.1f;
volatile int motor_duty_percent = 0;
volatile int encoder_setpoint = 0; 

// --- Estructura de Paquete UART ---
typedef struct {
    uint8_t start_byte;
    int8_t motor_direction; 
    int32_t raw_encoder_ticks;
    uint8_t end_byte;
} __attribute__((packed)) MotorDataPacket_t;

// -------------------------------------------------------------------
// FUNCIONES DE CONFIGURACIÓN
// -------------------------------------------------------------------

void setup_pwm() {
    gpio_set_direction(MOTOR_PIN_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_PIN_IN2, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_PIN_IN1, 0);
    gpio_set_level(MOTOR_PIN_IN2, 0);

    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE, .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = PWM_RESOLUTION, .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_ena = {
        .speed_mode     = LEDC_MODE, .channel        = LEDC_CHANNEL_ENA,
        .timer_sel      = LEDC_TIMER_0, .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MOTOR_PIN_PWM_ENA, .duty           = 0, .hpoint           = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_ena));
}

void setup_uart() {
    uart_config_t uart_config = {
        .baud_rate = 115200, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT
    };
    uart_driver_install(UART_NUM_0, 256 * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_0, &uart_config);
}

void setup_encoder() {
    pcnt_unit_config_t unit_config = {
        .low_limit = -32768,
        .high_limit = 32767,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 10000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_channel_handle_t pcnt_chan = NULL;
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = ENCODER_PIN_A,   // Pulso
        .level_gpio_num = ENCODER_PIN_B,  // Nivel para dirección
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    // Incrementa en flanco de subida, decrementa en flanco de bajada
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));

    // Mantiene la dirección según nivel del pin B
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}


// -------------------------------------------------------------------
// FUNCIÓN DE CONTROL DE MOTOR
// -------------------------------------------------------------------

void set_motor_speed(int duty_percent) {
    int max_duty = (1 << PWM_RESOLUTION) - 1;
    int new_duty_value = 0;
    
    if (duty_percent > 0) {
        new_duty_value = (int)((float)duty_percent / 100.0 * max_duty);
        gpio_set_level(MOTOR_PIN_IN1, 1); 
        gpio_set_level(MOTOR_PIN_IN2, 0);
    } else if (duty_percent < 0) {
        new_duty_value = (int)((float)abs(duty_percent) / 100.0 * max_duty);
        gpio_set_level(MOTOR_PIN_IN1, 0);
        gpio_set_level(MOTOR_PIN_IN2, 1); 
    } else {
        gpio_set_level(MOTOR_PIN_IN1, 0); 
        gpio_set_level(MOTOR_PIN_IN2, 0);
        new_duty_value = 0;
    }
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_ENA, new_duty_value);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_ENA);
}

// -------------------------------------------------------------------
// TAREAS DEL FREE RTOS
// -------------------------------------------------------------------

void encoder_task(void *pvParameters) {
    MotorDataPacket_t data_packet = { .start_byte = 0xA5, .end_byte = 0x5A };
    char tx_buffer[64];

    // Usamos una variable local para la cuenta anterior para calcular la dirección
    int previous_ticks = 0; 

    while(1) {
        int current_ticks = 0;
        
        // 1. Lectura de Ticks: Se usa un cast (int*) para eliminar la advertencia 'volatile'
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, (int*)&encoder_ticks));
        current_ticks = encoder_ticks;

        data_packet.raw_encoder_ticks = current_ticks;

        // 2. Determinación de Dirección (por comparación de conteo)
        if (current_ticks > previous_ticks) {
            data_packet.motor_direction = 1; // Avance
        } else if (current_ticks < previous_ticks) {
            data_packet.motor_direction = -1; // Reversam
        } else {
            data_packet.motor_direction = 0; // Estático
        }
        
        // 3. Actualizar la cuenta anterior para el siguiente ciclo
        previous_ticks = current_ticks;
        
        // 4. Envío de datos
        int len = snprintf(tx_buffer, sizeof(tx_buffer), "D:%d,T:%d\n", data_packet.motor_direction, current_ticks);
        uart_write_bytes(UART_NUM_0, tx_buffer, len); 

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void motor_control_task(void *pvParameters) {
    uint8_t rx_buffer[32];
    int rx_index = 0;
    const int control_loop_ms = 50;

    while (1) {
        // --- Control de Posición Proporcional ---
        int error = encoder_setpoint - encoder_ticks;
        int duty_to_apply = (int)(error * proportional_gain);

        if (error > -10 && error < 10) {
            duty_to_apply = 0;
        } else {
            // Saturar límite máximo y mínimo absoluto
            if (duty_to_apply > 100) {
                duty_to_apply = 100;
            } else if (duty_to_apply < -100) {
                duty_to_apply = -100;
            } else if (duty_to_apply >= 0 && duty_to_apply < MIN_PWM_DUTY) {
                duty_to_apply = MIN_PWM_DUTY;
            } else if (duty_to_apply <= 0 && duty_to_apply > -MIN_PWM_DUTY) {
                duty_to_apply = -MIN_PWM_DUTY;
            }
        }

        motor_duty_percent = duty_to_apply;
        set_motor_speed(motor_duty_percent);

        // --- Recepción UART (Setpoint) ---
        int rx_bytes = uart_read_bytes(UART_NUM_0, rx_buffer + rx_index, 1, pdMS_TO_TICKS(0));
        
        if (rx_bytes > 0) {
            char received_char = rx_buffer[rx_index];

            if (received_char == '\n' || received_char == '\r') {
                rx_buffer[rx_index] = '\0';

                if (strncmp((const char*)rx_buffer, "KP:", 3) == 0) {
                    float new_kp = atof((const char*)rx_buffer + 3);
                    if (new_kp > 0 && new_kp < 5) { // Pon rango seguro
                        proportional_gain = new_kp;
                    }
                } else {
                    sscanf((const char *)rx_buffer, "%d", (int*)&encoder_setpoint);
                }
                rx_index = 0;
            } else {
                rx_index++;
                if (rx_index >= sizeof(rx_buffer) - 1) {
                    rx_index = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(control_loop_ms));
    }
}

void app_main(void) {
    setup_pwm();
    setup_uart();
    setup_encoder();
    
    xTaskCreate(motor_control_task, "motor_control", 4096, NULL, 5, NULL);
    xTaskCreate(encoder_task, "encoder_read", 2048, NULL, 5, NULL);
}