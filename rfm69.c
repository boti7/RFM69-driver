#include "rfm69_port.h"
#include "rfm69_config.h"
#include "rfm69.h"

// Register mask for R/W operation
#define R_REGISTER  0x7F
#define W_REGISTER  0x80

RFM69_Data_t received_data = {0};
void (*recv_cb)(RFM69_Data_t) = NULL;
void (*send_cb)(void) = NULL;
RFM69_Mode_t current_mode;

static uint8_t register_read(uint8_t reg_addr);
static void register_write(uint8_t reg_addr, uint8_t value);

/**
 * Initialize the radio module
 *
 * @return 0 if initializaton succeeded, 1 if there was an error
 */
uint8_t rfm69_init()
{
    uint8_t i;

    // Test the communication with writeing to SyncValue1 then reading it back 
    i = 0;
    do register_write(0x2F, 0xAA); while (register_read(0x2F) != 0xAA && ++i < 50);
    if (i >= 50) return 1; // timeout, return with error
    i = 0;
    do register_write(0x2F, 0x55); while (register_read(0x2F) != 0x55 && ++i < 50);
    if (i >= 50) return 1; // timeout, return with error

    // Write configuration to the radio
    for (i = 0; rfm69_configuration[i][0] != 0xFF && i < 0xFF; i++)
    {
        register_write(rfm69_configuration[i][0], rfm69_configuration[i][1]);
    }

    return 0;
}

/**
 * Set the radio operation mode
 *
 * @param mode New operation mode
 */
uint8_t rfm69_set_mode(RFM69_Mode_t mode)
{
    if (mode == RX)
    {
        register_write(0x3D, (register_read(0x3D) & 0xFB) | 0x04);  // restart RX
        register_write(0x25, 0x40);                                 // use DIO0 as PayloadReady
    }

    if (mode == TX)
    {
        register_write(0x25, 0x00);                                 // use DIO0 as PacketSent
    }

    register_write(0x01, (register_read(0x01) & 0xE3) | mode);      // change OpMode

    // Wait for ModeReady
    uint8_t i = 0;
    while ((register_read(0x27) & 0x80) == 0x00 && ++i < 50);
    if (i >= 50) return 1; // timeout, return with error

    current_mode = mode;

    return 0;
}

/**
 * Set a callback function which will be called when data arrives
 *
 * @param cb Callback function
 */
void rfm69_set_receive_callback(void (*cb)(RFM69_Data_t))
{
    recv_cb = cb;
}

/**
 * Set a callback function which will be called when sending finished
 *
 * @param cb Callback function
 */
void rfm69_set_send_callback(void (*cb)(void))
{
    send_cb = cb;
}

/**
 * Call this function when the DIO0 pin causes an interrupt
 */
void rfm69_interrupt_handler()
{
    if (current_mode == RX && register_read(0x28) & 0x04)     // PayloadReady
    {
        rfm69_set_mode(STANDBY);

        SPI_SELECT();
        SPI_TRANSFER(R_REGISTER & 0x00);
        received_data.len = SPI_TRANSFER(0);
        received_data.len = received_data.len > MAX_PAYLOAD_LEN ? MAX_PAYLOAD_LEN : received_data.len - 3;
        received_data.target_id = SPI_TRANSFER(0);
        received_data.sender_id = SPI_TRANSFER(0);

        uint8_t ctl_byte = SPI_TRANSFER(0);
        received_data.ack_requested = ctl_byte & 0x40;
        received_data.ack_received = ctl_byte & 0x80;

        uint8_t i;
        for (i = 0; i < received_data.len; i++)
        {
            received_data.payload[i] = SPI_TRANSFER(0);
        }
        received_data.payload[i] = 0;

        SPI_UNSELECT();

        received_data.rssi = -1 * (register_read(0x24) / 2);

        if (recv_cb != NULL)
        {
            (*recv_cb)(received_data);
        }
    }

    if (current_mode == TX)
    {
        rfm69_set_mode(STANDBY);

        if (send_cb != NULL)
        {
            (*send_cb)();
        }
    }
}

/**
 * Send data
 */
void rfm69_send(uint8_t target_id, const void* payload, uint8_t payload_len, uint8_t request_ack)
{
    rfm69_set_mode(RX);
    
    uint16_t timeout = 1000;
    while ((-1 * (register_read(0x24) / 2)) > -90 && timeout > 0)
    {
        timeout--;
        DELAY_MS(1);
    }

    rfm69_set_mode(STANDBY);

    payload_len = payload_len > MAX_PAYLOAD_LEN ? MAX_PAYLOAD_LEN : payload_len;

    SPI_SELECT();
    SPI_TRANSFER(W_REGISTER | 0x00);
    SPI_TRANSFER(payload_len + 3);
    SPI_TRANSFER(target_id);
    SPI_TRANSFER(ADDRESS);
    SPI_TRANSFER(0);    // CTL

    uint8_t i;
    for (i = 0; i < payload_len; i++)
    {
        SPI_TRANSFER(((uint8_t*) payload)[i]);
    }

    SPI_UNSELECT();

    rfm69_set_mode(TX);
}

static uint8_t register_read(uint8_t reg_addr)
{
    SPI_SELECT();
    SPI_TRANSFER(R_REGISTER & reg_addr);
    uint8_t value = SPI_TRANSFER(0);
    SPI_UNSELECT();
    return value;
}

static void register_write(uint8_t reg_addr, uint8_t value)
{
    SPI_SELECT();
    SPI_TRANSFER(W_REGISTER | reg_addr);
    SPI_TRANSFER(value);
    SPI_UNSELECT();
}
