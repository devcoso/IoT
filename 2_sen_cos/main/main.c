#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

// Define la estructura de datos que se enviará
typedef struct {
    uint8_t start_byte;
    float seno;
    float coseno;
    uint8_t end_byte;
} __attribute__((packed)) DataPacket_t;

void app_main(void)
{
    // Configura UART0 para la comunicación serial
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    DataPacket_t data;
    float fase_rad = 0.0;
    const float incremento_fase = 0.1; // Ajusta este valor para cambiar la frecuencia
    data.start_byte = 0xA5; // Byte de inicio
    data.end_byte = 0x5A;   // Byte de fin

    while (1) {
        // Calcula los valores de seno y coseno en tiempo real
        data.seno = sin(fase_rad);
        data.coseno = cos(fase_rad);

        // Envía el paquete de datos completo a través de UART
        // Se envía un puntero a la estructura con su tamaño
        uart_write_bytes(UART_NUM_0, (const char *)&data, sizeof(DataPacket_t));

        // Incrementa la fase para la siguiente iteración
        fase_rad += incremento_fase;
        if (fase_rad >= 2 * M_PI) {
            fase_rad -= 2 * M_PI;
        }

        // Espera un momento antes de la siguiente transmisión
        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms de espera
    }
}