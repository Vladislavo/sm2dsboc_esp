#include <Arduino.h>
#include <SD.h>

#include "bus_protocol/bus_protocol.h"

#include <DHT.h>

#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "esp_wpa2.h"

#include "soc/rtc_wdt.h"

#define STORAGE_TYPE_UDP_SERVER             1
#define STORAGE_TYPE_SD                     2

#define STORAGE_TYPE                        STORAGE_TYPE_UDP_SERVER

#define UDP_SERVER_ADDRESS                  IPAddress(192,168,4,1)
#define UDP_SERVER_PORT                     40001

#define WIFI_SSID                           "iotlab"
#define WIFI_PASSWORD                       "conectate"

#define BUS_PROTOCOL_MAX_DATA_SIZE          128
#define BUS_PROTOCOL_MAX_WAITING_TIME       500

#define BAUDRATE                            115200
#define DHT11_PIN                           22
#define SOIL_MOISTER_ONBOARD_PIN            32
#define SOIL_MOISTER_EXT_PIN                33

#define DHT11_READ_RETRIES                  100

#define WIS_TX_PIN                          17
#define WIS_RX_PIN                          16
#define MKR_TX_PIN                          26
#define MKR_RX_PIN                          27

#define LED_STORAGE                         12
#define LED_SERIAL_RECEIVE                  13

#define DATA_SEND_PERIOD                    600000 //10 min

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
    float sht85_temp = 0;
    float sht85_hum = 0;
    float hih8121_temp = 0;
    float hih8121_hum = 0;
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

void store_data(const sensors_data_t *sensors_data);
void dataup_udp_server(const sensors_data_t *sensors_data);

void bus_protocol_data_send_decode(
    sensors_data_t *sensors_data,
    const uint8_t *buffer,
    const uint8_t buffer_length);
uint8_t bus_protocol_serial_receive(Stream *serial, uint8_t *data, uint8_t *data_length, const uint32_t timeout);

void sleep_mcu(const uint32_t ms);

SemaphoreHandle_t xBinarySemaphore;
static volatile uint8_t esp_taking_dht_data = 0;

DHT dht(DHT11_PIN, DHT11);

WiFiUDP clientUDP;
NTPClient timeClient(clientUDP);

static const char dev_names[][12]  = {"wisper_node", "mkr1000", "esp32"};

void setup() {
    Serial.begin(BAUDRATE);
    
    bus_wis.begin(BAUDRATE, SERIAL_8N1, WIS_RX_PIN, WIS_TX_PIN);
    bus_mkr.begin(BAUDRATE, SERIAL_8N1, MKR_RX_PIN, MKR_TX_PIN);

    pinMode(LED_SERIAL_RECEIVE, OUTPUT);
    digitalWrite(LED_SERIAL_RECEIVE, LOW);
    pinMode(LED_STORAGE, OUTPUT);
    digitalWrite(LED_STORAGE, LOW);

#if STORAGE_TYPE == STORAGE_TYPE_SD
    SD.begin();
#endif

    dht.begin();

    WiFi.disconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    uint8_t cnt = 60;
    while (WiFi.status() != WL_CONNECTED && cnt) {
        delay(500);
        Serial.print(".");
        cnt--;
    }
    if (!cnt) {
        Serial.println("WiFi not connected -> restart!");
        ESP.restart();
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address set: ");
    Serial.println(WiFi.localIP()); //print LAN IP

#if STORAGE_TYPE == STORAGE_TYPE_SD
    timeClient.begin();
    timeClient.setTimeOffset(1);
#elif STORAGE_TYPE == STORAGE_TYPE_UDP_SERVER
    clientUDP.begin(UDP_SERVER_PORT);
#endif

    rtc_wdt_set_length_of_reset_signal(RTC_WDT_SYS_RESET_SIG, RTC_WDT_LENGTH_3_2us);
    rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_SYSTEM);
    rtc_wdt_set_time(RTC_WDT_STAGE0, 250);
    
    rtc_wdt_protect_off();
    rtc_wdt_disable();

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
    if (WiFi.status() != WL_CONNECTED) {
        delay(60000);
        if (WiFi.status() != WL_CONNECTED) {
            ESP_LOGE(TAG, "WIFI NOT CONNECTED DURING LAST MIN >> BOARD RESTART");
            ESP.restart();
        }
    }
}

void sensors_task (void *parameter) {
    sensors_data_t sensors_data;
    sensors_data.board_id = BUS_PROTOCOL_BOARD_ID_ESP;
    
    for(;;) {
        // read data
        read_sensors_data(&sensors_data);

        // write new sd record
        store_data(&sensors_data);
        
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
                    digitalWrite(LED_SERIAL_RECEIVE, HIGH);
                    ESP_LOGD(TAG, "WIS DATA SEND");
                    ESP_LOGD(TAG, "%d bytes", buffer_length);
                    
                    bus_protocol_data_send_decode(&sensors_data, buffer, buffer_length);

                    bus_protocol_packet_encode(BUS_PROTOCOL_PACKET_TYPE_ACK, buffer, 0, buffer, &buffer_length);
                    bus_wis.write(buffer, buffer_length);

                    digitalWrite(LED_SERIAL_RECEIVE, LOW);
                    // write new sd record 
                    store_data(&sensors_data);
                default:
                    break;
            }
        }
        sleep_mcu(3);
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
                    digitalWrite(LED_SERIAL_RECEIVE, HIGH);
                    ESP_LOGD(TAG, "MKR DATA SEND");
                    ESP_LOGD(TAG, "%d bytes\r\n", buffer_length);
                    
                    bus_protocol_data_send_decode(&sensors_data, buffer, buffer_length);

                    // ACK data send
                    bus_protocol_packet_encode(BUS_PROTOCOL_PACKET_TYPE_ACK, buffer, 0, buffer, &buffer_length);
                    bus_mkr.write(buffer, buffer_length);

                    digitalWrite(LED_SERIAL_RECEIVE, LOW);
                    // write new sd record
                    store_data(&sensors_data);
                default:
                    break;
            }
        }
        buffer_length = 0;
        sleep_mcu(3);
    }
}

void read_sensors_data(sensors_data_t *sensors_data) {
    uint8_t retries = DHT11_READ_RETRIES;
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
        retries--;
    } while ((isnan(sensors_data->dht_temp) || isnan(sensors_data->dht_hum)) && retries);
}

void sd_add_record(const sensors_data_t *sensors_data) {
    if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
        digitalWrite(LED_STORAGE, HIGH);
        char buf[256] = "";
        char filename[24] = "/sensors_";
        // redundancy file
        char filename_s[24] = "/sensors_s_";

        timeClient.update();

        ESP_LOGD(TAG, "writen sd: board_id = %d, time: %s, sm_0 = %u, sm_1 = %u, sm_2 = %u, t = %0.2f, h = %0.2f\r\n",
                        sensors_data->board_id, timeClient.getFormattedDate().c_str(), sensors_data->soil_moisture_0, 
                        sensors_data->soil_moisture_1, sensors_data->soil_moisture_2,
                        sensors_data->dht_temp, sensors_data->dht_hum);

        
        switch (sensors_data->board_id) {
            case BUS_PROTOCOL_BOARD_ID_ESP:
                sprintf(buf, "%s, %u, %u, %0.2f, %0.2f\r\n", 
                        timeClient.getFormattedDate().c_str(), sensors_data->soil_moisture_0, 
                        sensors_data->soil_moisture_1, sensors_data->dht_temp, sensors_data->dht_hum);

                strcat(filename, "esp.csv");
                strcat(filename_s, "esp.csv");
                break;
            case BUS_PROTOCOL_BOARD_ID_WIS:
                sprintf(buf, "%s, %u, %u, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\r\n", 
                        timeClient.getFormattedDate().c_str(), sensors_data->soil_moisture_0, 
                        sensors_data->soil_moisture_1, sensors_data->dht_temp, sensors_data->dht_hum,
                        sensors_data->sht85_temp, sensors_data->sht85_hum,
                        sensors_data->hih8121_temp, sensors_data->hih8121_hum);
                
                strcat(filename, "wis.csv");
                strcat(filename_s, "wis.csv");
                break;
            case BUS_PROTOCOL_BOARD_ID_MKR:
                sprintf(buf, "%s, %u, %u, %u, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\r\n", 
                        timeClient.getFormattedDate().c_str(), sensors_data->soil_moisture_0, 
                        sensors_data->soil_moisture_1, sensors_data->soil_moisture_2,
                        sensors_data->dht_temp, sensors_data->dht_hum,
                        sensors_data->sht85_temp, sensors_data->sht85_hum,
                        sensors_data->hih8121_temp, sensors_data->hih8121_hum);

                strcat(filename, "mkr.csv");
                strcat(filename_s, "mkr.csv");
                break;
            default:
                break;
        }

        File fp = SD.open(filename, FILE_APPEND);
        
        if (fp) {
            fp.printf(buf);
            fp.close();

            ESP_LOGD(TAG, "SD written\n");
        }

        if ((fp = SD.open(filename_s, FILE_APPEND))) {
            fp.printf(buf);
            fp.close();

            ESP_LOGD(TAG, "SD_s written\n");
        }

        digitalWrite(LED_STORAGE, LOW);
        xSemaphoreGive(xBinarySemaphore);
    }
}

void store_data(const sensors_data_t *sensors_data) {
#if STORAGE_TYPE == STORAGE_TYPE_UDP_SERVER
    dataup_udp_server(sensors_data);
#else
    sd_add_record(sensors_data);
#endif
}

void dataup_udp_server(const sensors_data_t *sensors_data) {
    if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
        digitalWrite(LED_STORAGE, HIGH);
        char buf[256] = "";

        snprintf(buf, sizeof(buf), "%s@", dev_names[sensors_data->board_id]);

        switch (sensors_data->board_id) {
            case BUS_PROTOCOL_BOARD_ID_ESP:
                snprintf(buf, sizeof(buf), "%s%u, %u, %0.2f, %0.2f\r\n",
                        buf,
                        sensors_data->soil_moisture_0, sensors_data->soil_moisture_1, 
                        sensors_data->dht_temp, sensors_data->dht_hum);
                break;
            case BUS_PROTOCOL_BOARD_ID_WIS:
                snprintf(buf, sizeof(buf), "%s%u, %u, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\r\n",
                        buf,
                        sensors_data->soil_moisture_0, sensors_data->soil_moisture_1, 
                        sensors_data->dht_temp, sensors_data->dht_hum,
                        sensors_data->sht85_temp, sensors_data->sht85_hum,
                        sensors_data->hih8121_temp, sensors_data->hih8121_hum);
                break;
            case BUS_PROTOCOL_BOARD_ID_MKR:
                snprintf(buf, sizeof(buf), "%s%u, %u, %u, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\r\n",
                        buf,
                        sensors_data->soil_moisture_0, sensors_data->soil_moisture_1, sensors_data->soil_moisture_2, 
                        sensors_data->dht_temp, sensors_data->dht_hum,
                        sensors_data->sht85_temp, sensors_data->sht85_hum,
                        sensors_data->hih8121_temp, sensors_data->hih8121_hum);
                break;
            default:
                break;
        }

        clientUDP.beginPacket(UDP_SERVER_ADDRESS, UDP_SERVER_PORT);
        clientUDP.print(buf);

        if (clientUDP.endPacket()) {
            ESP_LOGD(TAG, "Dataup complete!");
        } else {
            ESP_LOGE(TAG, "Dataup failed!");
        }

        digitalWrite(LED_STORAGE, LOW);
        xSemaphoreGive(xBinarySemaphore);
    }
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
        rtc_wdt_feed();
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

    if (sensors_data->board_id == BUS_PROTOCOL_BOARD_ID_MKR) {
        memcpy(&sensors_data->soil_moisture_2, &buffer[p_len], sizeof(sensors_data->soil_moisture_2));
        p_len += sizeof(sensors_data->soil_moisture_2);
    }

    memcpy(&sensors_data->dht_temp, &buffer[p_len], sizeof(sensors_data->dht_temp));
    p_len += sizeof(sensors_data->dht_temp);

    memcpy(&sensors_data->dht_hum, &buffer[p_len], sizeof(sensors_data->dht_hum));
    p_len += sizeof(sensors_data->dht_hum);

    memcpy(&sensors_data->sht85_temp, &buffer[p_len], sizeof(sensors_data->sht85_temp));
    p_len += sizeof(sensors_data->sht85_temp);

    memcpy(&sensors_data->sht85_hum, &buffer[p_len], sizeof(sensors_data->sht85_hum));
    p_len += sizeof(sensors_data->sht85_hum);

    memcpy(&sensors_data->hih8121_temp, &buffer[p_len], sizeof(sensors_data->hih8121_temp));
    p_len += sizeof(sensors_data->hih8121_temp);

    memcpy(&sensors_data->hih8121_hum, &buffer[p_len], sizeof(sensors_data->hih8121_hum));
    p_len += sizeof(sensors_data->hih8121_hum);
}