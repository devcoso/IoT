#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_log.h"

void app_main(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);

    float angulo;
    while(1) {
        int lectura = adc1_get_raw(ADC1_CHANNEL_0);

        // Escala invertida
        if (lectura > 1) {
            angulo = -0.06227*lectura + 254.30;
        } else {
            angulo = 270.0; // Valor máximo cuando la lectura es 0 o 1
        }

        ESP_LOGI("ADC", "Lectura: %d  --> Ángulo estimado: %.2f°", lectura, angulo);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
