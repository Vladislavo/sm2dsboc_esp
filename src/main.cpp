#include <Arduino.h>
#include <SD.h>

#include "bus_protocol/bus_protocol.h"

#include <DHT.h>

#define BUS_PROTOCOL_MAX_DATA_SIZE          32
#define BUS_PROTOCOL_MAX_WAITING_TIME       300

#define BAUDRATE                            115200
#define DHT11_PIN                           22
#define SOIL_MOISTER_ONBOARD_PIN            32
#define SOIL_MOISTER_EXT_PIN                33

#define WIS_TX_PIN                          17
#define WIS_RX_PIN                          16

#define DATA_SEND_PERIOD                    60000

HardwareSerial bus_wis(2);
HardwareSerial bus_mkr(1);

typedef struct {
    board_id_t board_id = BUS_PROTOCOL_BOARD_ID_UNKNOWN;
    uint32_t utc = 0;
    uint16_t soil_moisture_0 = 0;
    uint16_t soil_moisture_1 = 0;
    float dht_temp = 0;
    float dht_hum = 0;
} sensors_data_t;

typedef enum {
    BUS_STATE_ON_IDLE = 0,
    BUS_STATE_ON_TRANSIMSSION
} bus_state_t;

void sensors_task (void *parameter);
void bus_task (void *parameter);

void read_sensors_data(sensors_data_t *sensors_data);
void sd_add_record(const sensors_data_t *sensors_data);

uint8_t bus_protocol_serial_receive(Stream *serial, uint8_t *data, uint8_t *data_length);

void sleep_mcu(const uint32_t ms);


DHT dht(DHT11_PIN, DHT11);

sensors_data_t sensors_data;

void setup() {
    Serial.begin(BAUDRATE);
    bus_wis.begin(BAUDRATE, SERIAL_8N1, WIS_RX_PIN, WIS_TX_PIN);
    // bus_mkr.begin(9600, SERIAL_8N1, MKR_RX_PIN, MKR_TX_PIN);
  
    SD.begin();
    
    dht.begin();

    xTaskCreate(sensors_task,     /* Task function. */
                "sensors_task",   /* String with name of task. */
                10000,            /* Stack size in bytes. */
                NULL,             /* Parameter passed as input of the task */
                1,                /* Priority of the task. */
                NULL);            /* Task handle. */
  
    xTaskCreate(bus_task,         /* Task function. */
                "bus_task",       /* String with name of task. */
                10000,            /* Stack size in bytes. */
                NULL,             /* Parameter passed as input of the task */
                1,                /* Priority of the task. */
                NULL);            /* Task handle. */ 
    
    delay(1000);
}

void loop() {
    delay(1000);
}

void sensors_task (void *parameter) {
    while (1) {
        // read data
        read_sensors_data(&sensors_data);
        // write new sd record 
        delay(1000);
    }
}
 
void bus_task (void *parameter) {
    bus_state_t bus_state = BUS_STATE_ON_IDLE;
    board_id_t talking_with = BUS_PROTOCOL_BOARD_ID_UNKNOWN;

    uint8_t buffer[BUS_PROTOCOL_MAX_DATA_SIZE] = {0};
    uint8_t buffer_length = 0;

    while (1) {
        if (bus_protocol_serial_receive(&bus_wis, buffer, &buffer_length)) {
            // Serial.print(F("Received message: "));
            // for (uint8_t i = 0; i < buffer_length; i++) {
            //     Serial.printf("%02X", buffer[i]);
            // }
            // Serial.println();

            switch (bus_protocol_packet_decode(buffer, buffer_length, buffer, &buffer_length)) {
                case BUS_PROTOCOL_PACKET_TYPE_TRANSMIT_REQUEST :
                    Serial.println(F("WIS TRANSMIT REQUEST"));
                    talking_with = bus_protocol_transmit_request_decode(buffer, buffer_length);
                    
                    bus_protocol_transmit_grant_encode(talking_with, buffer, &buffer_length);
                    bus_wis.write(buffer, buffer_length);
                    
                    bus_state = BUS_STATE_ON_TRANSIMSSION;
                    break;
                case BUS_PROTOCOL_PACKET_TYPE_DATA_SEND :
                    Serial.println(F("WIS DATA SEND"));
                    Serial.printf("%d bytes\r\n", buffer_length);
                    if (bus_protocol_data_send_decode(  &sensors_data.board_id,
                                                        &sensors_data.utc,
                                                        &sensors_data.soil_moisture_0,
                                                        &sensors_data.soil_moisture_1,
                                                        &sensors_data.dht_temp,
                                                        &sensors_data.dht_hum,
                                                        buffer,
                                                        buffer_length)) 
                    {
                        // ACK data send
                        bus_protocol_packet_encode(BUS_PROTOCOL_PACKET_TYPE_ACK, buffer, 0, buffer, &buffer_length);
                        bus_wis.write(buffer, buffer_length);

                        // write sd
                        sd_add_record(&sensors_data);
                    }

                    bus_state = BUS_STATE_ON_IDLE;
                    talking_with = BUS_PROTOCOL_BOARD_ID_UNKNOWN;
                default:
                    break;
            }
        }
    }
}

void read_sensors_data(sensors_data_t *sensors_data) {
    sensors_data->soil_moisture_0 = analogRead(SOIL_MOISTER_ONBOARD_PIN);
    sensors_data->soil_moisture_1 = analogRead(SOIL_MOISTER_EXT_PIN);

    float hum = dht.readHumidity();
    // Read temperature as Celsius (the default)
    float temp = dht.readTemperature();

    sensors_data->dht_temp = temp;
    sensors_data->dht_hum = hum;

    // Check if any reads failed and exit early (to try again).
    if (isnan(hum) || isnan(temp)) {
      Serial.println(F("Failed to read from DHT sensor!"));
    }

    // Serial.println(F("Sensors data:"));
    // Serial.println(sensors_data->soil_moisture_0);
    // Serial.println(sensors_data->soil_moisture_1);
    // Serial.println(sensors_data->dht_temp);
    // Serial.println(sensors_data->dht_hum);
}

void sd_add_record(const sensors_data_t *sensors_data) {
    Serial.printf("writen sd: board_id = %d, utc = %u, sm_0 = %u, sm_1 = %u, t = %0.2f, h = %0.2f\r\n",
                    sensors_data->board_id, sensors_data->utc, sensors_data->soil_moisture_0, 
                    sensors_data->soil_moisture_1, sensors_data->dht_temp, sensors_data->dht_hum);
    File fp = SD.open("/sensor_data.csv", FILE_APPEND);
    
    if (fp) {
        fp.printf("%d, %u, %u, %u, %0.2f, %0.2f\r\n", 
                    sensors_data->board_id, sensors_data->utc, sensors_data->soil_moisture_0, 
                    sensors_data->soil_moisture_1, sensors_data->dht_temp, sensors_data->dht_hum);
        fp.close();

        Serial.println(F("SD written"));
    }
}

uint8_t bus_protocol_serial_receive(Stream *serial, uint8_t *data, uint8_t *data_length) {
    *data_length = 0;
    uint32_t start_millis = millis();
    while(start_millis + BUS_PROTOCOL_MAX_WAITING_TIME > millis() && *data_length < BUS_PROTOCOL_MAX_DATA_SIZE) {
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