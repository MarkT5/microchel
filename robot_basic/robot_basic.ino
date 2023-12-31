#include "Radio.h"
#include "MotorControl.h"

uint32_t pipes[4] = {0x787878787844, 0x787878787866, 0x787878787899, 0x787878787800};

Radio radio(RECEIVER, pipes[0], 7, 8);
MotorControl motor2(A3, A2, 6); //A3 - for one of //A1 - for the rest
MotorControl motor1(4, 3, 5);

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

  //motor1.write(100);
  analogWrite(9, max(100, min(255, msg.servo2))); //400 - 1023
  analogWrite(10, max(73, min(212, msg.servo1)));// 290 - 850
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
    motor1.write(0);
    motor2.write(0);
  }
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