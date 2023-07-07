
#include <SPI.h>
#include <RF24.h>
#include "Radio.h"



struct MSG{
  int motor1 = 230;
  int motor2 = 180;
  int servo1 = 25;
  int servo2 = 46;
  uint8_t command = 0;
  bool status = 1;
};

struct RSP{
  float heading = 0;
};

uint32_t pipes[4] = {0x787878787844, 0x787878787866, 0x787878787899, 0x787878787800};
Radio radio(TRANSMITTER, pipes[0]);
MSG msg;
RSP rsp;

void setup() {
  Serial.begin(115200);

  // RADIO
  Serial.println("Transmitter ON");
  radio.setRecvVar(&rsp, sizeof(rsp));
  radio.setSendVar(&msg, sizeof(msg));
  radio.init();
  attachInterrupt(0, rcheck, LOW);

}

void loop() {
  for (int i = 0; i < 4; i++){
    radio.changePipe(pipes[i]);
    radio.update();
  }
  delay(50);
}


void rcheck(){
  if (!radio.check_radio()){
    //Serial.println("radio fail");
  }
}
