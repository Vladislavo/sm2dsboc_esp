#include <Arduino.h>
#include <SD.h>

#include "bus_protocol/bus_protocol.h"

#include <DHT.h>

#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "esp_wpa2.h"

#define WIFI_SSID                           "eduroam"
#define EAP_IDENTITY                        "al373630@uji.es"
#define EAP_PASSWORD                        "estudianterykov"

#define BUS_PROTOCOL_MAX_DATA_SIZE          32
#define BUS_PROTOCOL_MAX_WAITING_TIME       500

#define BAUDRATE                            115200
#define DHT11_PIN                           22
#define SOIL_MOISTER_ONBOARD_PIN            32
#define SOIL_MOISTER_EXT_PIN                33

#define WIS_TX_PIN                          17
#define WIS_RX_PIN                          16
#define MKR_TX_PIN                          26
#define MKR_RX_PIN                          27

#define DATA_SEND_PERIOD                    60000

HardwareSerial bus_wis(2);
HardwareSerial bus_mkr(1);

typedef struct {
    board_id_t board_id = BUS_PROTOCOL_BOARD_ID_UNKNOWN;
    uint32_t utc = 0;
    uint16_t soil_moisture_0 = 0;
    uint16_t soil_moisture_1 = 0;
    uint16_t soil_moisture_2 = 0;
    float dht_temp = 0;
    float dht_hum = 0;
} sensors_data_t;

typedef enum {
    BUS_STATE_ON_IDLE = 0,
    BUS_STATE_ON_TRANSIMSSION
} bus_state_t;

void sensors_task (void *parameter);
void bus_task_wis (void *parameter);
void bus_task_mkr (void *parameter);

void read_sensors_data(sensors_data_t *sensors_data);
void sd_add_record(const sensors_data_t *sensors_data);

void bus_protocol_data_send_decode(
    sensors_data_t *sensors_data,
    const uint8_t *buffer,
    const uint8_t buffer_length);
uint8_t bus_protocol_serial_receive(Stream *serial, uint8_t *data, uint8_t *data_length, const uint32_t timeout);

void sleep_mcu(const uint32_t ms);

SemaphoreHandle_t xBinarySemaphore;
static volatile uint8_t esp_taking_dht_data = 0;

DHT dht(DHT11_PIN, DHT11);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void setup() {
    Serial.begin(BAUDRATE);
    
    bus_wis.begin(BAUDRATE, SERIAL_8N1, WIS_RX_PIN, WIS_TX_PIN);
    bus_mkr.begin(BAUDRATE, SERIAL_8N1, MKR_RX_PIN, MKR_TX_PIN);

    SD.begin();
    
    dht.begin();

    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY)));
    ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_username((uint8_t *)EAP_IDENTITY, strlen(EAP_IDENTITY)));
    ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_set_password((uint8_t *)EAP_PASSWORD, strlen(EAP_PASSWORD)));
    WiFi.enableSTA(true);

    esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_sta_wpa2_ent_enable(&config));

    WiFi.begin(WIFI_SSID);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address set: ");
    Serial.println(WiFi.localIP()); //print LAN IP

    timeClient.begin();
    timeClient.setTimeOffset(1);

    xBinarySemaphore = xSemaphoreCreateMutex();

    xTaskCreate(sensors_task,     /* Task function. */
                "sensors_task",   /* String with name of task. */
                10000,            /* Stack size in bytes. */
                NULL,             /* Parameter passed as input of the task */
                1,                /* Priority of the task. */
                NULL);            /* Task handle. */
  
    xTaskCreate(bus_task_wis,     /* Task function. */
                "bus_task_wis",   /* String with name of task. */
                10000,            /* Stack size in bytes. */
                NULL,             /* Parameter passed as input of the task */
                1,                /* Priority of the task. */
                NULL);            /* Task handle. */
    
    xTaskCreate(bus_task_mkr,     /* Task function. */
                "bus_task_mkr",   /* String with name of task. */
                10000,            /* Stack size in bytes. */
                NULL,             /* Parameter passed as input of the task */
                1,                /* Priority of the task. */
                NULL);            /* Task handle. */ 

    delay(1000);
}

void loop() {

}

void sensors_task (void *parameter) {
    sensors_data_t sensors_data;
    sensors_data.board_id = BUS_PROTOCOL_BOARD_ID_ESP;
    
    for(;;) {
        // read data
        read_sensors_data(&sensors_data);

        // write new sd record
        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
            sd_add_record(&sensors_data);

            xSemaphoreGive(xBinarySemaphore);
        }
        
        vTaskDelay((DATA_SEND_PERIOD + random(5000)) / portTICK_PERIOD_MS);
    }
}
 
void bus_task_wis (void *parameter) {
    sensors_data_t sensors_data;

    uint8_t buffer[BUS_PROTOCOL_MAX_DATA_SIZE] = {0};
    uint8_t buffer_length = 0;

    for(;;) {
        while (esp_taking_dht_data) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        if (bus_protocol_serial_receive(&bus_wis, buffer, &buffer_length, BUS_PROTOCOL_MAX_WAITING_TIME)) {
            switch (bus_protocol_packet_decode(buffer, buffer_length, buffer, &buffer_length)) {
                case BUS_PROTOCOL_PACKET_TYPE_DATA_SEND :
                    ESP_LOGD(TAG, "WIS DATA SEND");
                    ESP_LOGD(TAG, "%d bytes", buffer_length);
                    
                    bus_protocol_data_send_decode(&sensors_data, buffer, buffer_length);

                    bus_protocol_packet_encode(BUS_PROTOCOL_PACKET_TYPE_ACK, buffer, 0, buffer, &buffer_length);
                    bus_wis.write(buffer, buffer_length);

                    // write new sd record 
                    if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
                        sd_add_record(&sensors_data);

                        xSemaphoreGive(xBinarySemaphore);
                    }
                default:
                    break;
            }
        }
        sleep(3);
    }
}

void bus_task_mkr (void *parameter) {
    sensors_data_t sensors_data;

    uint8_t buffer[BUS_PROTOCOL_MAX_DATA_SIZE] = {0};
    uint8_t buffer_length = 0;

    for(;;) {
        while (esp_taking_dht_data) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
        if (bus_protocol_serial_receive(&bus_mkr, buffer, &buffer_length, BUS_PROTOCOL_MAX_WAITING_TIME)) {
            switch (bus_protocol_packet_decode(buffer, buffer_length, buffer, &buffer_length)) {
                case BUS_PROTOCOL_PACKET_TYPE_DATA_SEND :
                    ESP_LOGD(TAG, "MKR DATA SEND");
                    ESP_LOGD(TAG, "%d bytes\r\n", buffer_length);
                    
                    bus_protocol_data_send_decode(&sensors_data, buffer, buffer_length);

                    // ACK data send
                    bus_protocol_packet_encode(BUS_PROTOCOL_PACKET_TYPE_ACK, buffer, 0, buffer, &buffer_length);
                    bus_mkr.write(buffer, buffer_length);

                    // write new sd record 
                    if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
                        sd_add_record(&sensors_data);

                        xSemaphoreGive(xBinarySemaphore);
                    }
                default:
                    break;
            }
        }
        buffer_length = 0;
        sleep(3);
    }
}

void read_sensors_data(sensors_data_t *sensors_data) {
    sensors_data->soil_moisture_0 = analogRead(SOIL_MOISTER_ONBOARD_PIN);
    sensors_data->soil_moisture_1 = analogRead(SOIL_MOISTER_EXT_PIN);

    do {
        esp_taking_dht_data = 1;
        delay(100);
        sensors_data->dht_temp = dht.readTemperature();
        sensors_data->dht_hum = dht.readHumidity();
        esp_taking_dht_data = 0;
        if (isnan(sensors_data->dht_temp) || isnan(sensors_data->dht_hum)) {
            ESP_LOGE(TAG, "Failed to read from DHT sensor!\n");
            delay(100);
        }
    } while (isnan(sensors_data->dht_temp) || isnan(sensors_data->dht_hum));
}

void sd_add_record(const sensors_data_t *sensors_data) {
    timeClient.update();
    ESP_LOGD(TAG, "writen sd: board_id = %d, time: %s, sm_0 = %u, sm_1 = %u, sm_2 = %u, t = %0.2f, h = %0.2f\r\n",
                    sensors_data->board_id, timeClient.getFormattedDate().c_str(), sensors_data->soil_moisture_0, 
                    sensors_data->soil_moisture_1, sensors_data->soil_moisture_2,
                    sensors_data->dht_temp, sensors_data->dht_hum);
    portDISABLE_INTERRUPTS();
    File fp = SD.open("/sensor_data.csv", FILE_APPEND);
    
    if (fp) {
        fp.printf("%d, %s, %u, %u, %u, %0.2f, %0.2f\r\n", 
                    sensors_data->board_id, timeClient.getFormattedDate().c_str(), sensors_data->soil_moisture_0, 
                    sensors_data->soil_moisture_1, sensors_data->soil_moisture_2, 
                    sensors_data->dht_temp, sensors_data->dht_hum);
        fp.close();

        ESP_LOGD(TAG, "SD written\n");
    }
    portENABLE_INTERRUPTS();
}

uint8_t bus_protocol_serial_receive(Stream *serial, uint8_t *data, uint8_t *data_length, const uint32_t timeout) {
    *data_length = 0;
    uint32_t start_millis = millis();
    while(start_millis + timeout > millis() && *data_length < BUS_PROTOCOL_MAX_DATA_SIZE) {
        if (serial->available()) {
            data[(*data_length)++] = serial->read();
            // update wating time
            start_millis = millis();
        }
    }

    return *data_length;
}

void sleep_mcu(const uint32_t ms) {
    // future possible employ powerdown functions
    delay(ms);
}

void bus_protocol_data_send_decode(
    sensors_data_t *sensors_data,
    const uint8_t *buffer,
    const uint8_t buffer_length)
{
    uint8_t p_len = 0;

    sensors_data->board_id = (board_id_t) buffer[p_len];
    p_len++;

    memcpy(&sensors_data->utc, &buffer[p_len], sizeof(sensors_data->utc));
    p_len += sizeof(sensors_data->utc);

    memcpy(&sensors_data->soil_moisture_0, &buffer[p_len], sizeof(sensors_data->soil_moisture_0));
    p_len += sizeof(sensors_data->soil_moisture_0);

    memcpy(&sensors_data->soil_moisture_1, &buffer[p_len], sizeof(sensors_data->soil_moisture_1));
    p_len += sizeof(sensors_data->soil_moisture_1);

    memcpy(&sensors_data->soil_moisture_2, &buffer[p_len], sizeof(sensors_data->soil_moisture_2));
    p_len += sizeof(sensors_data->soil_moisture_2);

    memcpy(&sensors_data->dht_temp, &buffer[p_len], sizeof(sensors_data->dht_temp));
    p_len += sizeof(sensors_data->dht_temp);

    memcpy(&sensors_data->dht_hum, &buffer[p_len], sizeof(sensors_data->dht_hum));
    p_len += sizeof(sensors_data->dht_hum);
}