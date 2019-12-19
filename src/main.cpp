#include <Arduino.h>

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
    uint16_t soil_moisture_0 = 0;
    uint16_t soil_moisture_1 = 0;
    float dht_temp = 0;
    float dht_hum = 0;
} sensors_data_t;


void sensors_task (void *parameter);
void bus_task (void *parameter);

void read_sensors_data(sensors_data_t *sensors_data);
void sd_add_record(const sensors_data_t *sensors_data);

uint8_t bus_protocol_serial_receive(Stream *serial, uint8_t *data, uint8_t *data_length);

void sleep_mcu(const uint32_t ms);


DHT dht(DHT11_PIN, DHT11);

sensors_data_t sensors_data;

void setup() {
    Serial.begin(112500);
    bus_wis.begin(9600, SERIAL_8N1, WIS_RX_PIN, WIS_TX_PIN);
    // bus_wis.begin(9600, SERIAL_8N1, MKR_RX_PIN, MKR_TX_PIN);
  
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
    uint8_t buffer[BUS_PROTOCOL_MAX_DATA_SIZE] = {0};
    uint8_t buffer_length = 0;
    while (1) {
        // Serial.println(F("bus_task"));
        // delay(1000);
        if (bus_protocol_serial_receive(&bus_wis, buffer, &buffer_length)) {
            Serial.print(F("Received message: "));
            for (uint8_t i = 0; i < buffer_length; i++) {
                Serial.print(buffer[i], HEX);
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

    Serial.println(F("Sensors data:"));
    Serial.println(sensors_data->soil_moisture_0);
    Serial.println(sensors_data->soil_moisture_1);
    Serial.println(sensors_data->dht_temp);
    Serial.println(sensors_data->dht_hum);
}

uint8_t bus_protocol_serial_receive(Stream *serial, uint8_t *data, uint8_t *data_length) {
    *data_length = 0;
    uint32_t start_millis = millis();
    while(start_millis + BUS_PROTOCOL_MAX_WAITING_TIME < millis() && *data_length < BUS_PROTOCOL_MAX_DATA_SIZE) {
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