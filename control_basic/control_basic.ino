
#include <SPI.h>
#include <RF24.h>
#include "Radio.h"



struct MSG{
  int motor1 = 0;
  int motor2 = 0;
  int servo1;
  int servo2;
  uint8_t command = 0;
  bool status = 1;
};

struct RSP{
  float heading = 0;
};


Radio radio(TRANSMITTER, 0x7878787878LL);
MSG msg;
RSP rsp;

float rec;

void setup() {
  Serial.begin(115200);

  // RADIO
  Serial.println("Transmitter ON");
  radio.setRecvVar(&rsp, sizeof(rsp));
  radio.setSendVar(&msg, sizeof(msg));
  radio.init();
  attachInterrupt(0, rcheck, LOW);

}

char inChar;
int inpNum = 0;
byte inp_counter = 0;
uint8_t mode = 0;
bool start = true;

void loop() {
  radio.update();
  if (Serial.available()>0){
    inChar = Serial.read();
    if (start){
      start = false;
      switch(inChar){
        case 'l':
          inp_counter=0;
          mode = 1;
          break;
        case 'r':
          inp_counter=0;
          mode = 2;
          break;
        case 'u':
          inp_counter=0;
          mode = 3;
          break;
        case 'g':
          inp_counter=0;
          mode = 4;
          break;
        default:
          start = true;
      }
      
    }else{
      inpNum = (int)(inpNum << 8 | (byte)inChar);
      inp_counter+=1;
    }
    if (inp_counter > 1){
      switch(mode){
        case 1:
          msg.motor1 = inpNum;
          break;
        case 2:
          msg.motor2 = inpNum;
          break;
        case 3:
          msg.servo1 = inpNum;
          break;
        case 4:
          msg.servo2 = inpNum;
          break;
      }
      mode = 0;
      inpNum = 0;
      inp_counter = 0;
      start = true;
    }
  }
}


void rcheck(){
  if (!radio.check_radio()){
    //Serial.println("radio fail");
  }
}
