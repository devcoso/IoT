#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_log.h"

// Definiciones para el ADC
#define ADC_CHANNEL         ADC1_CHANNEL_0    // Pin VP (GPIO36)

void app_main(void)
{
    // --- 1. Configuración de UART0 ---
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    // --- 2. Configuración de ADC ---
    // Configura el ancho de la lectura a 12 bits (0-4095)
    adc1_config_width(ADC_WIDTH_BIT_12);
    // Configura la atenuación (esto define el rango máximo de voltaje que corresponde a 4095)
    // 11dB ATTENUATION = rango de 0V a ~3.3V
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);

    char buffer[10]; 
    
    while (1) {
        // --- 3. Lectura del valor crudo (raw) ---
        int raw_reading = adc1_get_raw(ADC_CHANNEL);
        
        // --- 4. Envío del Entero por UART ---
        // Formatea el entero 'raw_reading' (0-4095) en una cadena de texto (string).
        // El formato "%d" es para enteros. Agregamos el salto de línea \n.
        int len = snprintf(buffer, sizeof(buffer), "%d\n", raw_reading);

        // Envía el string (ej: "2048\n") por UART0.
        uart_write_bytes(UART_NUM_0, buffer, len);

        // Pausa de 500 ms
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}