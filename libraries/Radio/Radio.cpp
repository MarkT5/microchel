#include "Radio.h"


void Radio::init() {
    time = millis();
    radio.begin();                // инициализация
    delay(50);
    radio.setDataRate(RF24_1MBPS); // скорость обмена данными RF24_1MBPS или RF24_2MBPS
    radio.setCRCLength(RF24_CRC_8); // размер контрольной суммы 8 bit или 16 bit
    radio.setPALevel(RF24_PA_MAX); // уровень питания усилителя RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH and RF24_PA_MAX
    radio.setChannel(0x6f);         // установка канала
    radio.enableAckPayload();       // We will be using the ACK Payload feature which is not enabled by default
    radio.enableDynamicPayloads();
    radio.powerUp();               // включение или пониженное потребление powerDown - powerUp
    if (role == RECEIVER) {
        radio.openReadingPipe(1, pipe);
        radio.startListening();        // приём
        //radio.writeAckPayload(1, &heading_res, sizeof(heading_res)); // load the payload for the next time
    } else {
        radio.openWritingPipe(pipe);
    }
}


void Radio::update() {
    unsigned long curr_time = millis();
    if (role == RECEIVER) {
        if (curr_time-time > 300) {
            time = curr_time;
            reset();
        }
    } else {
        if (msg_delivered || curr_time-time > 100) {
            time = curr_time;
            msg_delivered = false;
            radio.startWrite(send_ptr, send_size, 0);
        }
        if (fail_counter > 4) {
            reset();
            fail_counter = 0;
        }
    }
}

bool Radio::check_radio() {
    bool tx, fail, rx;                 // declare variables to store IRQ flags
    radio.whatHappened(tx, fail, rx);  // What happened?
    if (fail) {
        last_status = false;
        fail_counter+=1;
        return false;
    }
    if (rx) {
        last_status = true;
        if (role == RECEIVER && radio.available()) {
            radio.read(recv_ptr, recv_size);
            radio.writeAckPayload(1, send_ptr, send_size); // load the payload for the next time
            time = millis();
        } else if (role == TRANSMITTER && radio.isAckPayloadAvailable()) {
            fail_counter = 0;
            msg_delivered = true;
            radio.read(recv_ptr, recv_size);
        }
        return true;
    }
}
