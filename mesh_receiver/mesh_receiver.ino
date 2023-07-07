#include "Radio.h"
#include "MotorControl.h"
#include "IMU.h"
#include "EEPROM.h"

Radio radio(RECEIVER, 0x787878787844, 7, 8);
IMU imu;
MotorControl motor1(A3, A2, 6); //A3 - for one of //A1 - for the rest
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

bool curr_A_pin = 0;
bool radio_status = false;
MSG msg;
RSP rsp;

void setup() {
  Serial.begin(115200);
  imu.init();
  EEPROM.begin();
  EEPROM.get(0, imu.m_min);
  EEPROM.get(sizeof(imu.m_min)+1, imu.m_max);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
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
  radio.update();
  imu.update();

  if (radio_status){
    Serial.print(msg.motor1);
    Serial.print('\t');
    Serial.print(msg.motor2);
    Serial.print('\t');
    Serial.print(msg.servo1);
    Serial.print('\t');
    Serial.println( max(290, min(850, msg.servo2)));
    motor1.write(msg.motor1);
    motor2.write(msg.motor2);
  }else{
    Serial.println("fail");
  }
}

void rcheck(){
  radio_status = radio.check_radio();
}