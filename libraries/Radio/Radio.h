#include <Arduino.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#define RECEIVER 1
#define TRANSMITTER 0

class Radio {
private:
    bool role;
    uint32_t pipe;

    void *recv_ptr;
    uint8_t recv_size;
    void *send_ptr;
    uint8_t send_size;
    RF24 radio;
    unsigned long time;
    bool msg_delivered = true;


public:
    bool last_status = true;
    uint8_t fail_counter = 0;

    Radio(bool r, uint32_t p, uint8_t pin1=9, uint8_t pin2=10) : role{r}, pipe{p} {
        radio = RF24(pin1, pin2);
    }

    void setRecvVar(void *ptr, uint8_t size) {
        recv_ptr = ptr;
        recv_size = size;
    }

    void setSendVar(void *ptr, uint8_t size) {
        send_ptr = ptr;
        send_size = size;
    }

    void reset() {
        radio.powerDown();
        init();
        radio.powerUp();
    }

    void update();

    bool check_radio(void);

    void init();
};