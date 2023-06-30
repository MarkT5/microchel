#include "Radio.h"
#include "MotorControl.h"
#include "directADC.h"
#include "IMU.h"

Radio radio(RECEIVER, 0x7878787878LL, 7, 8);
IMU imu;
MotorControl motor1(A0, A1, 6);
MotorControl motor2(4, 3, 5);

class Control{
  public:
    uint16_t X = 0;
    uint16_t Y = 0;
};

struct MSG{
  Control joystick;
  int servo1;
  int servo2;
  uint8_t command;
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
unsigned int last_reset = millis();
Control joystick;
Battery battery;
MSG msg;
RSP rsp;

void setup() {
  Serial.begin(115200);
  imu.init();

  //ADC (battery)
  ADC_enable(); // вызывается обязательно
  ADC_setPrescaler(64); // без вызова - делитель 2
  ADC_setReference(ADC_VCC); // без вызова - ADC_AREF
  ADC_attachInterrupt(adcReady); // добавим прерывание готовности
  setAnalogMux(ADC_A0); 
  ADC_startConvert();
  
  // RADIO
  Serial.println("ReceiverTester ON");
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

float target_ang = 0;
float curr_ang = 0;
float ang_int = 0;
unsigned long last_time = micros();
float delta_ang = 0;

void loop() {
  float ang_diff = target_ang-curr_ang;
  if (ang_diff > 180){
    ang_diff-=360;
  }else if (ang_diff < -180){
    ang_diff+=360;
  }
  
  Serial.print(curr_ang);
  Serial.print('\t');
  Serial.print(target_ang);
  Serial.print('\t');
  Serial.print(ang_diff);
  Serial.print('\t');
  Serial.println(ang_int);

  if (!isnanf(ang_diff)){
    ang_int+=ang_diff*0.01;
    if (ang_int > 100){
      ang_int=100;
    }else if (ang_int < -100){
      ang_int=-100;
    }
  }
  if (abs(ang_diff)>60 || abs(ang_diff)<5){
    ang_int=0;
  }
  
  rsp.counter+=1;
  radio.update();
  imu.update();
  if (msg.command == 1){
    calibrate_imu();
  }
  rsp.heading = imu.heading_res;
  if (rsp.heading > 180){
    rsp.heading-=360;
  }else if (rsp.heading < -180){
    rsp.heading+=360;
  }
  if (!isnanf(curr_ang)){
    // Serial.print(curr_ang);
    // Serial.print('\t');
    // Serial.print(rsp.heading);
    // Serial.print('\t');
    // Serial.println(micros()-last_time);
    if(curr_ang != rsp.heading){
      float res = curr_ang-rsp.heading;
      if (res > 180){
        res = 360 - res;
      }else if (res < -180){
        res = -360 - res;
      }
      delta_ang = res*12000/(micros()-last_time);
    }
  }
  last_time = micros();
  curr_ang = imu.heading_res;
  if (curr_ang > 180){
    curr_ang-=360;
  }else if (curr_ang < -180){
    curr_ang+=360;
  }
  //motor1.write(100);
  analogWrite(9, max(100, msg.servo1/4)); //400 - 1023
  analogWrite(10, max(73, min(212, msg.servo2/4)));// 290 - 850
  float fov = (int(msg.joystick.X)-502)/2.0;
  float side = (int(msg.joystick.Y)-497)/2.0;
  if (abs(side/100.0)>0.05){
    target_ang += side/100.0;
  }

  if (target_ang > 180){
    target_ang-=360;
  }else if (target_ang < -180){
    target_ang+=360;
  }
  
  //motor1.write(fov-side);
  //motor2.write(fov+side);
  if (abs(ang_diff)>5){
    motor1.write(fov+ang_diff*0.9+ang_int+delta_ang);
    motor2.write(-fov+ang_diff*0.9+ang_int+delta_ang);
  }else{
    motor1.write(fov);
    motor2.write(-fov);
  }
  rsp.curr_ang = curr_ang;
  rsp.target_ang = target_ang;
  rsp.fov = fov;
  rsp.ang_diff = ang_diff;
  rsp.ang_int = ang_int;
  rsp.delta_ang = delta_ang;
}

void calibrate_imu(){
  motor1.beep(30, 500);
  delay(100);
  motor1.beep(30, 500);
  delay(100);
  imu.calibrate();
  motor1.beep(30, 500);
  delay(100);
  motor1.beep(30, 500);
  delay(100);
  motor1.beep(30, 500);
  delay(100);
}

void rcheck(){
  radio.check_radio();
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