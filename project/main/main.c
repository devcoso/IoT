#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
// No se incluye driver/pulse_cnt.h

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

// --- Variables Globales ---
volatile int motor_duty_percent = 0;
volatile int encoder_setpoint = 0; // Almacena el duty cycle recibido por UART

// --- Estructura de Paquete UART (MODIFICADA) ---
// El paquete mantiene su tamaño original (7 bytes) para no romper la sincronización
typedef struct {
    uint8_t start_byte;
    int8_t motor_direction;  
    
    // *** NUEVOS CAMPOS ***
    int8_t a_level;         // Estado de Pin A (0 o 1)
    int8_t b_level;         // Estado de Pin B (0 o 1)
    // *********************

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

void setup_encoder_gpio() {
    // Configura los pines del encoder como entradas GPIO con pull-up.
    gpio_set_direction(ENCODER_PIN_A, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ENCODER_PIN_A, GPIO_PULLUP_ONLY);

    gpio_set_direction(ENCODER_PIN_B, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ENCODER_PIN_B, GPIO_PULLUP_ONLY);
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
    // La estructura ahora se inicializa con 0s, asegurando que 'reserved' sea 0.
    MotorDataPacket_t data_packet = { 
        .start_byte = 0xA5, 
        .end_byte = 0x5A
    };
    
    while(1) {
        // 1. Lectura digital pura y asignación directa a los nuevos campos
        data_packet.a_level = (int8_t)gpio_get_level(ENCODER_PIN_A);
        data_packet.b_level = (int8_t)gpio_get_level(ENCODER_PIN_B);

        // 2. Dirección del motor (del comando de control)
        if (motor_duty_percent > 0) {
            data_packet.motor_direction = 1; // Avance
        } else if (motor_duty_percent < 0) {
            data_packet.motor_direction = -1; // Reversa
        } else {
            data_packet.motor_direction = 0; // Estático
        }
        
        // 3. Envío de datos binarios
        uart_write_bytes(UART_NUM_0, (const char *)&data_packet, sizeof(MotorDataPacket_t));
        
        // Muestreo rápido (200 Hz)
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void motor_control_task(void *pvParameters) {
    uint8_t rx_buffer[32];
    int rx_index = 0;
    const int control_loop_ms = 50;
    
    while (1) {
        // Aplicar el último duty cycle recibido por UART
        int duty_to_apply = encoder_setpoint; 
        
        // Saturación
        if (duty_to_apply > 100) duty_to_apply = 100;
        else if (duty_to_apply < -100) duty_to_apply = -100;

        motor_duty_percent = duty_to_apply;
        set_motor_speed(motor_duty_percent);

        // --- Recepción UART (Nuevo Setpoint / Duty Cycle) ---
        int rx_bytes = uart_read_bytes(UART_NUM_0, rx_buffer + rx_index, 1, pdMS_TO_TICKS(0));
        
        if (rx_bytes > 0) {
            char received_char = rx_buffer[rx_index];

            if (received_char == '\n' || received_char == '\r') {
                rx_buffer[rx_index] = '\0';
                
                // Recibe el nuevo duty cycle (velocidad)
                sscanf((const char *)rx_buffer, "%d", (int*)&encoder_setpoint);
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
    setup_encoder_gpio(); // Configura A y B como entradas GPIO
    
    xTaskCreate(motor_control_task, "motor_control", 4096, NULL, 5, NULL);
    xTaskCreate(encoder_task, "encoder_read", 2048, NULL, 5, NULL);
}