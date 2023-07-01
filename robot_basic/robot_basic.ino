#include "Radio.h"
#include "MotorControl.h"
#include "directADC.h"
#include "IMU.h"
#include "EEPROM.h"

Radio radio(RECEIVER, 0x7878787878LL, 7, 8);
IMU imu;
MotorControl motor1(A1, A0, 6);
MotorControl motor2(4, 3, 5);

struct MSG{
  int motor1;
  int motor2;
  int servo1;
  int servo2;
  uint8_t command = 0;
  bool status = 1;
};

struct RSP{
  float heading = 0;
};

class Battery{
  public:
    float cel[2] = {0, 0};
    float low = 3.5;
    float floor = 3.2;
    void updateCel1(uint16_t val){
      cel[0] = val*5.0/1024-0.1;
    }
    void updateCel2(uint16_t val){
      cel[1] = val*5.0/1024*2-cel[0]-0.2;
    }
    bool check(){
      if (cel[0] < low || cel[0] < low){
        return false;
      }else{
        return true;
      }
    }
};

bool curr_A_pin = 0;
bool radio_status = false;
Battery battery;
MSG msg;
RSP rsp;

void setup() {
  // Serial.begin(115200);
  imu.init();
  EEPROM.begin();
  EEPROM.get(0, imu.m_min);
  EEPROM.get(sizeof(imu.m_min)+1, imu.m_max);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  //ADC (battery)
  ADC_enable(); // вызывается обязательно
  ADC_setPrescaler(64); // без вызова - делитель 2
  ADC_setReference(ADC_VCC); // без вызова - ADC_AREF
  ADC_attachInterrupt(adcReady); // добавим прерывание готовности
  setAnalogMux(ADC_A0); 
  ADC_startConvert();
  
  // RADIO
  //Serial.println("ReceiverTester ON");
  radio.setRecvVar(&msg, sizeof(msg));
  radio.setSendVar(&rsp, sizeof(rsp));
  radio.init();
  attachInterrupt(0, rcheck, LOW);

  //Servo
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  motor1.beep(30, 100);
  delay(100);
  motor1.beep(30, 100);
  delay(100);
  motor1.beep(30, 100);
  delay(100);
}


void loop() {
  if (!battery.check()){
    warning();
  }
  
  radio.update();
  imu.update();
  
  rsp.heading = imu.heading_res;
  if (msg.command == 1){
    calibrate_imu();
  }

  //motor1.write(100);
  analogWrite(9, max(100, min(255, msg.servo2))); //400 - 1023
  analogWrite(10, max(73, min(212, msg.servo1)));// 290 - 850
  if (radio_status){
    // Serial.print(msg.motor1);
    // Serial.print('\t');
    // Serial.print(msg.motor2);
    // Serial.print('\t');
    // Serial.print(msg.servo1);
    // Serial.print('\t');
    // Serial.println( max(290, min(850, msg.servo2)));
    motor1.write(msg.motor1);
    motor2.write(msg.motor2);
  }else{
    // Serial.println("fail");
    motor1.write(0);
    motor2.write(0);
  }
}

void calibrate_imu(){
  motor1.beep(30, 500);
  delay(100);
  motor1.beep(30, 500);
  delay(100);
  imu.calibrate();
  EEPROM.put(0, imu.m_min);
  EEPROM.put(sizeof(imu.m_min)+1, imu.m_max);
  motor1.beep(30, 500);
  delay(100);
  motor1.beep(30, 500);
  delay(100);
  motor1.beep(30, 500);
  delay(100);
}

void rcheck(){
  radio_status = radio.check_radio();
}

void warning(){
    motor1.beep(30, 100);
    delay(100);
    motor2.beep(30, 100);
    delay(100);
    motor1.beep(30, 100);
    delay(100);
    motor2.beep(30, 100);
    delay(100);
    motor1.beep(30, 100);
    delay(100);
    motor2.beep(30, 100);
    delay(100);
}

void adcReady(){
  if (curr_A_pin){
    battery.updateCel1(ADC_read());
    setAnalogMux(ADC_A2);
  }else{
    battery.updateCel2(ADC_read());
    setAnalogMux(ADC_A3);
  }
  curr_A_pin = !curr_A_pin;
  ADC_startConvert();
}