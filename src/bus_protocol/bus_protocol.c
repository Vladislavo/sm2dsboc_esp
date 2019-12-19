#include "bus_protocol.h"
#include <stdint.h>
#include <string.h>

typedef enum {
    START_FRAME_BYTE = 'S',
    END_FRAME_BYTE = 'E'
} frame_delimiter_t;

void bus_protocol_packet_encode(
    const packet_type_t packet_type,
    const uint8_t *data,
    const uint8_t data_length,
    uint8_t *packet,
    uint8_t *packet_length) 
{
    *packet_length = 0;
    
    packet[*packet_length] = START_FRAME_BYTE;
    (*packet_length)++;
    
    memcpy(&packet[*packet_length], data, data_length);
    (*packet_length) += data_length;

    packet[*packet_length] = END_FRAME_BYTE;
    (*packet_length)++;
}

packet_type_t bus_protocol_packet_decode(
    const uint8_t *packet,
    const uint8_t packet_length, 
    uint8_t *data,
    uint8_t *data_length)
{
    packet_type_t ret = BUS_PROTOCOL_PACKET_TYPE_UNKNOWN;

    if (packet[0] == START_FRAME_BYTE && packet[packet_length-1] == END_FRAME_BYTE) {
        ret = packet[1];
        memcpy(data, &packet[2], packet_length-3);

        *data_length = packet_length - 3;
    }

    return ret;
}

void bus_protocol_transmit_request_encode(
    const board_id_t board_id, 
    uint8_t *packet, 
    uint8_t *packet_length)
{
    *packet_length = 0;

    packet[*packet_length] = START_FRAME_BYTE;
    (*packet_length)++;

    packet[*packet_length] = BUS_PROTOCOL_PACKET_TYPE_TRANSMIT_REQUEST;
    (*packet_length)++;

    packet[*packet_length] = board_id;
    (*packet_length)++;

    packet[*packet_length] = END_FRAME_BYTE;
    (*packet_length)++;
}

board_id_t bus_protocol_transmit_request_decode(
    const uint8_t *packet, 
    const uint8_t packet_length)
{
    return bus_protocol_transmit_grant_decode(packet, packet_length);
}

void bus_protocol_transmit_grant_encode(
    const board_id_t board_id, 
    uint8_t *packet, 
    uint8_t *packet_length)
{
    *packet_length = 0;

    packet[*packet_length] = START_FRAME_BYTE;
    (*packet_length)++;

    packet[*packet_length] = BUS_PROTOCOL_PACKET_TYPE_TRANSMIT_GRANT;
    (*packet_length)++;

    packet[*packet_length] = board_id;
    (*packet_length)++;

    packet[*packet_length] = END_FRAME_BYTE;
    (*packet_length)++;
}

board_id_t bus_protocol_transmit_grant_decode(
    const uint8_t *packet, 
    const uint8_t packet_length)
{
    return packet_length == 1 && packet[0] < BUS_PROTOCOL_BOARD_ID_UNKNOWN? 
            packet[0] : BUS_PROTOCOL_BOARD_ID_UNKNOWN;
}

void bus_protocol_data_send_encode(
    const board_id_t board_id, 
    const uint32_t utc, 
    const uint16_t soil_moisture_0, 
    const uint16_t soil_moisture_1, 
    const float dht_temp, 
    const float dht_hum,
    uint8_t *packet,
    uint8_t *packet_length) 
{
    *packet_length = 0;

    packet[*packet_length] = START_FRAME_BYTE;
    (*packet_length)++;

    packet[*packet_length] = BUS_PROTOCOL_PACKET_TYPE_DATA_SEND;
    (*packet_length)++;

    packet[*packet_length] = board_id;
    (*packet_length)++;

    memcpy(&packet[*packet_length], &utc, sizeof(utc));
    (*packet_length) += sizeof(utc);

    memcpy(&packet[*packet_length], &soil_moisture_0, sizeof(soil_moisture_0));
    (*packet_length) += sizeof(soil_moisture_0);

    memcpy(&packet[*packet_length], &dht_temp, sizeof(dht_temp));
    (*packet_length) += sizeof(dht_temp);

    memcpy(&packet[*packet_length], &dht_hum, sizeof(dht_hum));
    (*packet_length) += sizeof(dht_hum);

    packet[*packet_length] = END_FRAME_BYTE;
    (*packet_length)++;
}

uint8_t bus_protocol_data_send_decode(
    board_id_t *board_id, 
    uint32_t *utc, 
    uint16_t *soil_moisture_0, 
    uint16_t *soil_moisture_1, 
    float *dht_temp, 
    float *dht_hum,
    const uint8_t *packet,
    const uint8_t packet_length) 
{
    uint8_t p_len = 0;

    if (packet[0] == START_FRAME_BYTE && packet[packet_length-1] == END_FRAME_BYTE) {
        memcpy(board_id, &packet[p_len], sizeof(*board_id));
        p_len++;

        memcpy(utc, &packet[p_len], sizeof(*utc));
        p_len += sizeof(*utc);

        memcpy(soil_moisture_0, &packet[p_len], sizeof(*soil_moisture_0));
        p_len += sizeof(*soil_moisture_0);

        memcpy(soil_moisture_1, &packet[p_len], sizeof(*soil_moisture_1));
        p_len += sizeof(*soil_moisture_1);

        memcpy(dht_temp, &packet[p_len], sizeof(*dht_temp));
        p_len += sizeof(*dht_temp);

        memcpy(dht_hum, &packet[p_len], sizeof(*dht_hum));
        p_len += sizeof(*dht_hum);
    }

    return p_len == packet_length;
}