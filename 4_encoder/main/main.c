#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h" // Se usa el driver moderno (pulse_cnt.h)
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
#define ENCODER_PIN_A (25)
#define ENCODER_PIN_B (26)
// La constante COUNTS_PER_OUTPUT_REVOLUTION ya no se usa directamente en el envío
#define COUNTS_PER_OUTPUT_REVOLUTION (1496.0f)

static const char *TAG = "MOTOR_ENCODER";

// *** NUEVO MANEJADOR DEL PCNT ***
pcnt_unit_handle_t pcnt_unit = NULL;

// *** CORRECCIÓN: Usamos 'int' (32 bits) para la lectura PCNT (compatible con el API) ***
volatile int encoder_ticks = 0; 
volatile int motor_duty_percent = 0;

// Define la NUEVA estructura de datos que se enviará
typedef struct {
    uint8_t start_byte;
    int8_t motor_direction; 
    int32_t raw_encoder_ticks; // Mandamos los ticks brutos como entero (4 bytes)
    uint8_t end_byte;
} __attribute__((packed)) MotorDataPacket_t;

// -------------------------------------------------------------------
// FUNCIONES DE CONFIGURACIÓN Y CONTROL (LEDC/PWM y UART)
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
        .gpio_num       = MOTOR_PIN_PWM_ENA, .duty           = 0, .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_ena));
    ESP_LOGI(TAG, "PWM configurado en el GPIO %d (ENA).", MOTOR_PIN_PWM_ENA);
    ESP_LOGI(TAG, "Pines de control (IN1, IN2) en los GPIOs %d y %d.", MOTOR_PIN_IN1, MOTOR_PIN_IN2);
}

void setup_uart() {
    uart_config_t uart_config = {
        .baud_rate = 115200, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_DEFAULT
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
        gpio_set_level(MOTOR_PIN_IN1, 1); 
        gpio_set_level(MOTOR_PIN_IN2, 0);
        ESP_LOGI(TAG, "Motor adelante: %d%%. Ciclo de trabajo: %d", duty_percent, new_duty_value);
    } else if (duty_percent < 0) {
        new_duty_value = (int)((float)abs(duty_percent) / 100.0 * max_duty);
        gpio_set_level(MOTOR_PIN_IN1, 0);
        gpio_set_level(MOTOR_PIN_IN2, 1); 
        ESP_LOGI(TAG, "Motor reversa: %d%%. Ciclo de trabajo: %d", abs(duty_percent), new_duty_value);
    } else {
        gpio_set_level(MOTOR_PIN_IN1, 0); 
        gpio_set_level(MOTOR_PIN_IN2, 0);
        new_duty_value = 0;
        ESP_LOGI(TAG, "Motor detenido.");
    }
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_ENA, new_duty_value);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_ENA);
}

// -------------------------------------------------------------------
// CONFIGURACIÓN DEL ENCODER (USANDO EL NUEVO DRIVER pulse_cnt.h)
// -------------------------------------------------------------------

void setup_encoder() {
    // 1. Configuración de la Unidad PCNT
    pcnt_unit_config_t unit_config = {
        .low_limit = -32768, // Límites de conteo típicos
        .high_limit = 32767,
        .intr_priority = 0,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    // 2. Configuración del Filtro (Debe ir en estado INIT)
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 10000, // 10 microsegundos de filtro
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    // 3. Configuración de los Canales (Cuadratura 4X)
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    pcnt_chan_config_t chan_config_a = { 
        .edge_gpio_num = ENCODER_PIN_A,
        .level_gpio_num = ENCODER_PIN_B,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config_a, &pcnt_chan_a));

    pcnt_channel_handle_t pcnt_chan_b = NULL;
    pcnt_chan_config_t chan_config_b = {
        .edge_gpio_num = ENCODER_PIN_B,
        .level_gpio_num = ENCODER_PIN_A,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config_b, &pcnt_chan_b));
    
    // 4. Configurar las acciones (Cuadratura)
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_INVERSE, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

    // 5. Habilitar e Iniciar
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit)); // Pasa a estado ENABLED
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit)); // Ahora puede iniciarse

    ESP_LOGI(TAG, "Encoder configurado y funcionando.");
}

// -------------------------------------------------------------------
// TAREAS DEL FREE RTOS
// -------------------------------------------------------------------

void encoder_task(void *pvParameters) {
    MotorDataPacket_t data_packet;
    data_packet.start_byte = 0xA5;
    data_packet.end_byte = 0x5A;

    while(1) {
        // Usamos la variable global 'encoder_ticks' (int) para la lectura.
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &encoder_ticks));
        
        // El paquete envía los ticks brutos (int32_t)
        data_packet.raw_encoder_ticks = encoder_ticks;

        // Determina la dirección del motor (basado en el comando PWM)
        if (motor_duty_percent > 0) {
            data_packet.motor_direction = 1; // Adelante
        } else if (motor_duty_percent < 0) {
            data_packet.motor_direction = -1; // Reversa
        } else {
            data_packet.motor_direction = 0; // Detenido
        }
        
        // Envía el paquete binario
        uart_write_bytes(UART_NUM_0, (const char *)&data_packet, sizeof(MotorDataPacket_t));
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void motor_control_task(void *pvParameters) {
    uint8_t rx_buffer[32];
    int rx_index = 0;
    
    while (1) {
        // Lee un byte a la vez
        int rx_bytes = uart_read_bytes(UART_NUM_0, rx_buffer + rx_index, 1, pdMS_TO_TICKS(10));
        if (rx_bytes > 0) {
            char received_char = rx_buffer[rx_index];
            if (received_char == '\n' || received_char == '\r') {
                rx_buffer[rx_index] = '\0';
                // Intenta convertir la cadena ASCII recibida
                if (sscanf((const char *)rx_buffer, "%d", (int*)&motor_duty_percent) == 1) {
                    if (motor_duty_percent >= -100 && motor_duty_percent <= 100) {
                        set_motor_speed(motor_duty_percent);
                    } else {
                        ESP_LOGW(TAG, "Valor fuera de rango (-100 a 100).");
                    }
                } else {
                    // Esta advertencia aparece cuando se reciben datos binarios del encoder
                    ESP_LOGW(TAG, "No se pudo convertir a entero. (Conflicto de UART con encoder)");
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