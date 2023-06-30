#include <SPI.h>
#include <RF24.h>
#include "directADC.h"
#include "Radio.h"


class Control{
  public:
    uint16_t X = 0;
    uint16_t Y = 0;
};

struct MSG{
  Control joystick;
  int servo1;
  int servo2;
  uint8_t command = 0;
  bool status = 1;
};

struct RSP{
  float heading = 0;
  uint8_t counter = 0;
  float curr_ang = 0;
  float target_ang = 0;
  float fov = 0;
  float ang_diff = 0;
  float ang_int = 0;
  float delta_ang = 0;
};


Radio radio(TRANSMITTER, 0x7878787878LL);
MSG msg;
RSP rsp;
float bat[2] = {0, 0};

uint8_t curr_A_pin = 0;

float rec;



void setup() {
  Serial.begin(115200);


  //ADC (joystick)
  ADC_enable(); // вызывается обязательно
  ADC_setPrescaler(64); // без вызова - делитель 2
  ADC_setReference(ADC_VCC); // без вызова - ADC_AREF
  ADC_attachInterrupt(adcReady); // добавим прерывание готовности
  setAnalogMux(ADC_A2); 
  ADC_startConvert();

  // RADIO
  Serial.println("Transmitter ON");
  radio.setRecvVar(&rsp, sizeof(rsp));
  radio.setSendVar(&msg, sizeof(msg));
  radio.init();
  attachInterrupt(0, rcheck, LOW);

}

void loop() {
  radio.update();
  // Serial.print(msg.joystick.X);
  // Serial.print(' ');
  // Serial.print(msg.joystick.Y);
  // Serial.print(' ');
  // Serial.print(msg.servo1);
  // Serial.print(' ');
  // Serial.println(msg.servo2);
  //Serial.println(msg.command);
  Serial.print(rsp.curr_ang);
  Serial.print('\t');
  Serial.print(rsp.target_ang);
  Serial.print('\t');
  Serial.print(rsp.fov);
  Serial.print('\t');
  Serial.print(rsp.ang_diff);
  Serial.print('\t');
  Serial.print(rsp.ang_int);
  Serial.print('\t');
  Serial.println(rsp.delta_ang);
  
  if (msg.servo2>1000){
    msg.command = 1;
  }else{
    msg.command = 0;
  }
  //Serial.print(joystick.X);Serial.print(' ');Serial.println(joystick.Y);
}

void adcReady(){
  if (curr_A_pin == 0){
    msg.joystick.X = ADC_read();
    setAnalogMux(ADC_A0);
    curr_A_pin = 1;
  }else if (curr_A_pin == 1){
    msg.joystick.Y = ADC_read();
    setAnalogMux(ADC_A6);
    curr_A_pin = 2;
  }else if (curr_A_pin == 2){
    msg.servo1 = ADC_read();
    setAnalogMux(ADC_A7);
    curr_A_pin = 3;
  }else if (curr_A_pin == 3){
    msg.servo2 = ADC_read();
    setAnalogMux(ADC_A2);
    curr_A_pin = 0;
  }
  
  ADC_startConvert();
}

void rcheck(){
  radio.check_radio();
}
