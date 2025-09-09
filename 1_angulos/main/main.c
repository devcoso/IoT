#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "driver/uart.h"

// Define the data packet structure
typedef struct {
    uint8_t start_byte;
    float angulo;
    uint8_t end_byte;
} __attribute__((packed)) DataPacket_t;

void app_main(void)
{
    // Configure UART0
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

    DataPacket_t data;
    data.start_byte = 0xA5;
    data.end_byte = 0x5A;

    while (1) {
        int lectura = adc1_get_raw(ADC1_CHANNEL_0);

        if (lectura > 1) {
            data.angulo = -0.06227 * lectura + 254.30;
        } else {
            data.angulo = 270.0;
        }

        // Send the complete data packet
        uart_write_bytes(UART_NUM_0, (const char *)&data, sizeof(DataPacket_t));

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}