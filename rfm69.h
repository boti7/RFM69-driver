#ifndef __RFM69_H__
#define __RFM69_H__

#include "rfm69_config.h"

// Frequency of the module
typedef enum
{
    FREQ_315MHZ,
    FREQ_433MHZ,
    FREQ_868MHZ,
    FREQ_915MHZ
} RFM69_Frequency_t;

// Radio operation modes
typedef enum
{
    SLEEP   = 0x00,
    STANDBY = 0x04,
    SYNTH   = 0x08,
    TX      = 0x0C,
    RX      = 0x10,
} RFM69_Mode_t;

// Received data
typedef struct
{
    uint8_t sender_id;
    uint8_t target_id;
    int16_t rssi;
    uint8_t payload[MAX_PAYLOAD_LEN + 1];
    uint8_t len;
    uint8_t ack_requested;
    uint8_t ack_received;
} RFM69_Data_t;

uint8_t rfm69_init();
uint8_t rfm69_set_mode(RFM69_Mode_t mode);
uint8_t rfm69_get_data(RFM69_Data_t *data);
void rfm69_set_receive_callback(void (*recv_cb)());
void rfm69_set_send_callback(void (*cb)(void));
void rfm69_interrupt_handler(void);
void rfm69_send(uint8_t target_id, const void* payload, uint8_t payload_len, uint8_t request_ack);

#endif
