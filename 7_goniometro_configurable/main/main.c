#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_log.h"

// Definiciones
#define ADC_CHANNEL         ADC1_CHANNEL_0    // Pin VP (GPIO36)
#define UART_NUM            UART_NUM_0
#define BUF_SIZE            (1024)

// Constantes de calibración iniciales (se actualizarán desde Node-RED)
static float m_pendiente = -0.06227; 
static float b_ordenada = 254.30; // Valor objetivo de 0 a 270

// Valores en porcentaje
// static float m_pendiente = -0.02348;
// static float b_ordenada = 95.22; // Valor objetivo de 0 a 100\
// -.7, 270

// --- Tarea para Recibir y Actualizar la Calibración (Rx Task) ---
void rx_task(void *arg)
{
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // Terminar cadena

            // Se espera formato: "m,b" (ej: "-0.06227,254.30")
            char *token;
            char *rest = (char *)data;

            // 1. Pendiente (m)
            token = strtok_r(rest, ",", &rest);
            if (token != NULL) {
                float nuevo_m = strtof(token, NULL);
                if (nuevo_m != 0.0) {
                    m_pendiente = nuevo_m;
                }
            }

            // 2. Ordenada (b)
            token = strtok_r(rest, ",", &rest);
            if (token != NULL) {
                float nuevo_b = strtof(token, NULL);
                if (nuevo_b != 0.0 || (strcmp(token, "0") == 0)) {
                    b_ordenada = nuevo_b;
                }
            }
        }
    }
    free(data);
}

// --- Tarea para Leer ADC, Calcular y Enviar Ángulo (Tx Task) ---
void tx_task(void *arg)
{
    char buffer[32]; 

    // Configuración del ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);

    while (1) {
        int raw_reading = adc1_get_raw(ADC_CHANNEL);
        float angulo = (raw_reading < 4095) ? (m_pendiente * raw_reading + b_ordenada) : 0.0;

        ESP_LOGI("ADC", "Raw: %d, Ángulo: %.2f", raw_reading, angulo);

        // Enviar por UART en formato legible
        // Ejemplo: "A:123.45\n"
        int len = snprintf(buffer, sizeof(buffer), "A:%.2f\n", angulo);
        uart_write_bytes(UART_NUM, buffer, len);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    // --- Configuración UART0 ---
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    // --- Creación de tareas ---
    xTaskCreate(rx_task, "uart_rx_task", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(tx_task, "adc_tx_task", 2048, NULL, configMAX_PRIORITIES - 2, NULL);
}
