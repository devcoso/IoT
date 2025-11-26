#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "string.h"

// Librerías de Red y MQTT de ESP-IDF
#include "esp_wifi.h"
#include "esp_event.h"
#include "mqtt_client.h"

// --- ⚙️ CONFIGURACIÓN DE RED Y MQTT ---
#define WIFI_SSID       "WIFI_SSID_EJEMPLO"
#define WIFI_PASSWORD   "WIFI_PASSWORD_EJEMPLO"
#define MQTT_SERVER_URI "mqtt://192.168.1.1:1883" // Usa tu IP y el puerto
#define MQTT_TOPIC_CMD  "comando/led"
#define LED_PIN         GPIO_NUM_2 // Pin GPIO 2 para el LED
// ------------------------------------

static const char *TAG = "MQTT_LED_DEMO";
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// --- 💡 CONTROL DEL LED ---
void led_control(int state) {
    gpio_set_level(LED_PIN, state);
    ESP_LOGI(TAG, "LED en GPIO %d cambiado a: %s", LED_PIN, (state == 1) ? "ON" : "OFF");
}

// --- FUNCIÓN DE CALLBACK PARA MENSAJES MQTT ---
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            // Suscripción al tópico de comando después de conectar
            esp_mqtt_client_subscribe(event->client, MQTT_TOPIC_CMD, 0); 
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA recibido. Tópico: %.*s", event->topic_len, event->topic);

            // Verificamos si es el tópico de control del LED
            if (event->topic_len == strlen(MQTT_TOPIC_CMD) && strncmp(event->topic, MQTT_TOPIC_CMD, event->topic_len) == 0) {
                // Node-RED ui_switch envía '1' (ON) o '0' (OFF) como payload.
                if (event->data_len > 0 && event->data[0] == '1') {
                    led_control(1); // Encender
                } else if (event->data_len > 0 && event->data[0] == '0') {
                    led_control(0); // Apagar
                }
            }
            break;
        default:
            ESP_LOGD(TAG, "Otro evento MQTT: %d", event_id);
            break;
    }
}

// --- TAREA MQTT ---
static void mqtt_app_start(void) {
    // La URI definida previamente es: #define MQTT_SERVER_URI "mqtt://192.168.4.43:1883"
    
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_SERVER_URI, // <-- Usar la estructura anidada
        //.session.protocol_ver = MQTT_PROTOCOL_V3_1_1, // Opcional, pero recomendado para compatibilidad
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

// --- MANEJADOR DE EVENTOS WIFI ---
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Reconectando a WiFi...");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Conectado. Dirección IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        mqtt_app_start(); // Iniciar MQTT después de obtener IP
    }
}

// --- SETUP DE WIFI ---
void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();

    // Configuración TCP/IP
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // Registro de manejadores de eventos
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    // Configuración Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .scan_method = WIFI_FAST_SCAN,
            //.sort_method = WIFI_SORT_RSSI,
            .threshold.rssi = -127,
            .pmf_cfg = {.capable = true, .required = false},
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "Inicialización Wi-Fi STA terminada.");
}

// --- FUNCIÓN PRINCIPAL app_main ---
void app_main(void) {
    // Inicializar NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1. Configurar el GPIO del LED
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    led_control(0); // Apagar el LED al inicio

    // 2. Iniciar la conexión WiFi
    wifi_init_sta();

    // Esperar la conexión es handled por el evento de IP_EVENT_STA_GOT_IP,
    // que luego llama a mqtt_app_start()
}