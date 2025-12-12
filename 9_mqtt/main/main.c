#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "mqtt_client.h"
#include "esp_crt_bundle.h" // <--- CRUCIAL PARA HIVEMQ

// Drivers del Motor y Encoder
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

// --- ⚙️ CREDENCIALES (MODIFICAR ESTO) ---
#define WIFI_SSID       "Wi-Fi IPN"
#define WIFI_PASSWORD   ""

// Tu Cluster URL (con mqtts:// y puerto 8883)
#define MQTT_SERVER_URI "mqtts://70e60bddcd3e4397a0835e6bedc33a72.s1.eu.hivemq.cloud:8883"
#define MQTT_USER       "esp32"
#define MQTT_PASS       "Patito81"

// --- TÓPICOS MQTT ---
#define TOPIC_TELEMETRY "motor/estado"   // ESP32 -> Nube (Publica posición)
#define TOPIC_SETPOINT  "motor/setpoint" // Nube -> ESP32 (Recibe objetivo)
#define TOPIC_KP        "motor/kp"       // Nube -> ESP32 (Ajusta ganancia)

// --- PINES DEL HARDWARE (IGUAL QUE TU CÓDIGO) ---
#define MOTOR_PIN_IN1       (2)
#define MOTOR_PIN_IN2       (4)
#define MOTOR_PIN_PWM_ENA   (5)
#define ENCODER_PIN_A       (25)
#define ENCODER_PIN_B       (26)

#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define PWM_FREQ_HZ         (5000)
#define PWM_RESOLUTION      (LEDC_TIMER_10_BIT)
#define LEDC_CHANNEL_ENA    LEDC_CHANNEL_0
#define MIN_PWM_DUTY        (40)

static const char *TAG = "MOTOR_MQTT";

// --- VARIABLES GLOBALES ---
pcnt_unit_handle_t pcnt_unit = NULL;
esp_mqtt_client_handle_t client = NULL; // Cliente MQTT global

// Variables de control (Volatile porque se cambian desde interrupciones/otras tareas)
volatile int encoder_ticks = 0; 
volatile float proportional_gain = 0.1f;
volatile int motor_duty_percent = 0;
volatile int encoder_setpoint = 0; 

// Grupo de eventos para Wi-Fi
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// -------------------------------------------------------------------
// 1. CONFIGURACIÓN DE HARDWARE (MOTOR Y ENCODER)
// -------------------------------------------------------------------

void setup_pwm() {
    gpio_set_direction(MOTOR_PIN_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_PIN_IN2, GPIO_MODE_OUTPUT);
    gpio_set_level(MOTOR_PIN_IN1, 0);
    gpio_set_level(MOTOR_PIN_IN2, 0);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE, .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RESOLUTION, .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel_ena = {
        .speed_mode = LEDC_MODE, .channel = LEDC_CHANNEL_ENA,
        .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_PIN_PWM_ENA, .duty = 0, .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_ena));
}

void setup_encoder() {
    pcnt_unit_config_t unit_config = {
        .low_limit = -32768, .high_limit = 32767, .intr_priority = 0,
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = { .max_glitch_ns = 10000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_channel_handle_t pcnt_chan = NULL;
    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = ENCODER_PIN_A, .level_gpio_num = ENCODER_PIN_B,
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_config, &pcnt_chan));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

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
// 2. LÓGICA MQTT (Recibir Setpoint y KP)
// -------------------------------------------------------------------

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    
    if (event->event_id == MQTT_EVENT_CONNECTED) {
        ESP_LOGI(TAG, "Conectado a MQTT. Suscribiendo...");
        esp_mqtt_client_subscribe(client, TOPIC_SETPOINT, 0);
        esp_mqtt_client_subscribe(client, TOPIC_KP, 0);
    } 
    else if (event->event_id == MQTT_EVENT_DATA) {
        // Copiar el payload a un buffer temporal para procesarlo seguro
        char data_buf[32];
        int len = event->data_len;
        if (len >= sizeof(data_buf)) len = sizeof(data_buf) - 1;
        memcpy(data_buf, event->data, len);
        data_buf[len] = '\0'; // Null-terminate

        // Chequear a qué tópico llegó
        if (strncmp(event->topic, TOPIC_SETPOINT, event->topic_len) == 0) {
            encoder_setpoint = atoi(data_buf); // Convertir a int
            ESP_LOGI(TAG, "Nuevo Setpoint recibido: %d", encoder_setpoint);
        } 
        else if (strncmp(event->topic, TOPIC_KP, event->topic_len) == 0) {
            proportional_gain = atof(data_buf); // Convertir a float
            ESP_LOGI(TAG, "Nuevo KP recibido: %.2f", proportional_gain);
        }
    }
}

static void mqtt_app_start(void) {
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address.uri = MQTT_SERVER_URI,
            .verification.crt_bundle_attach = esp_crt_bundle_attach, // Magia SSL
        },
        .credentials = {
            .username = MQTT_USER,
            .authentication.password = MQTT_PASS
        }
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

// -------------------------------------------------------------------
// 3. TAREAS RTOS
// -------------------------------------------------------------------

// Tarea: Lee encoder y ENVÍA datos por MQTT (Publica)
void encoder_task(void *pvParameters) {
    char json_payload[64];
    int previous_ticks = 0; 
    int motor_direction = 0;

    while(1) {
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, (int*)&encoder_ticks));
        
        // Calcular dirección localmente
        if (encoder_ticks > previous_ticks) motor_direction = 1;
        else if (encoder_ticks < previous_ticks) motor_direction = -1;
        else motor_direction = 0;
        previous_ticks = encoder_ticks;

        // Si estamos conectados, enviamos telemetría
        if (client != NULL) {
            // Formato JSON: {"ticks": 120, "dir": 1, "duty": 50}
            snprintf(json_payload, sizeof(json_payload), 
                     "{\"ticks\":%d,\"dir\":%d,\"duty\":%d}", 
                     encoder_ticks, motor_direction, motor_duty_percent);
                     
            esp_mqtt_client_publish(client, TOPIC_TELEMETRY, json_payload, 0, 0, 0);
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // Enviar cada 200ms para no saturar
    }
}

// Tarea: Control del Motor (Loop P)
void motor_control_task(void *pvParameters) {
    const int control_loop_ms = 50;

    while (1) {
        // 1. Calcular Error
        int error = encoder_setpoint - encoder_ticks;
        
        // 2. Controlador P
        int duty_to_apply = (int)(error * proportional_gain);

        // 3. Zona muerta y saturación
        if (error > -10 && error < 10) {
            duty_to_apply = 0; // Apagar si está cerca
        } else {
            if (duty_to_apply > 100) duty_to_apply = 100;
            if (duty_to_apply < -100) duty_to_apply = -100;
            // Garantizar potencia mínima para vencer fricción
            if (duty_to_apply >= 0 && duty_to_apply < MIN_PWM_DUTY) duty_to_apply = MIN_PWM_DUTY;
            if (duty_to_apply <= 0 && duty_to_apply > -MIN_PWM_DUTY) duty_to_apply = -MIN_PWM_DUTY;
        }

        // 4. Aplicar al hardware
        motor_duty_percent = duty_to_apply;
        set_motor_speed(motor_duty_percent);

        vTaskDelay(pdMS_TO_TICKS(control_loop_ms));
    }
}

// -------------------------------------------------------------------
// 4. INICIALIZACIÓN (Wi-Fi y Main)
// -------------------------------------------------------------------

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        mqtt_app_start(); // Iniciar MQTT al tener internet
    }
}

void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_OPEN, // Asumiendo red abierta IPN o celular
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void) {
    // Init NVS (Necesario para Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    setup_pwm();
    setup_encoder();

    // Arrancar comunicaciones
    wifi_init_sta();

    // Arrancar Tareas
    xTaskCreate(motor_control_task, "motor_ctrl", 4096, NULL, 5, NULL);
    xTaskCreate(encoder_task, "encoder_mqtt", 4096, NULL, 5, NULL);
}