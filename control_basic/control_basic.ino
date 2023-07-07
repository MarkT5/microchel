
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

uint32_t pipes[4] = {0x787878787844, 0x787878787866, 0x787878787899, 0x787878787800};
Radio radio(TRANSMITTER, pipes[4]);
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
int curr_robot = 0;
unsigned long last_serial_time = millis();
unsigned int serial_timout = 200;

void loop() {
  if (curr_robot < sizeof(pipes) / sizeof(uint32_t)){
    radio.changePipe(pipes[curr_robot]);
  }
  radio.update();
  if (Serial.available()>0){
    last_serial_time = millis();
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
        case 'm':
          inp_counter = 0;
          mode = 5;
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
        case 5:
          //msg.servo1 = inpNum;
          curr_robot = inpNum;
          break;
      }
      mode = 0;
      inpNum = 0;
      inp_counter = 0;
      start = true;
    }
  }else if (millis() - last_serial_time > serial_timout){
    last_serial_time = millis();
    Serial.println("clear");
    Serial.println(millis() - last_serial_time);
    clear_msg();
    for (uint32_t pipe : pipes){
       radio.changePipe(pipe);
       radio.update();
    }
  }
}


void rcheck(){
  if (!radio.check_radio()){
    //Serial.println("radio fail");
  }
}


void clear_msg(){
  msg.motor1 = 0;
  msg.motor2 = 0;
  // msg.servo1 = 100;
  // msg.servo2 = 130;
  // msg.command = 0;
  // msg.status = 1;
}
