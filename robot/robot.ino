#include "Radio.h"
#include "MotorControl.h"
#include "directADC.h"
#include "IMU.h"
#include "EEPROM.h"

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
    float low = 3.8;
    float floor = 3.2;
    void updateCel1(uint16_t val){
      cel[0] = val*5.0/1024-0.1;
    }
    void updateCel2(uint16_t val){
      cel[1] = val*5.0/1024*2-cel[0]-0.2;
    }
    bool check(){
      if (cel[0] < low || cel[1] < low){
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
  //Serial.begin(115200);
  imu.init();
  EEPROM.begin();
  EEPROM.get(0, imu.m_min);
  EEPROM.get(sizeof(imu.m_min)+1, imu.m_max);
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
  if (!battery.check()){
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
    return;
  }
  
  // Serial.print(imu.m.x);
  // Serial.print('\t');
  // Serial.print(imu.m.y);
  // Serial.print('\t');
  // Serial.print(imu.m.z);
  // Serial.print('\t');
  // Serial.println(imu.m.z);
//Serial.println(battery.cel[0]);
//Serial.println(battery.cel[1]);

  if (!isnanf(ang_diff)){
    
    ang_int+=ang_diff*0.01;
    if (ang_int > 100){
      ang_int=100;
    }else if (ang_int < -100){
      ang_int=-100;
    }
  }
  if(abs(ang_diff)<5){
      ang_int = 0;
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
    if(imu.mag_ready and imu.acc_ready and rsp.heading != curr_ang){
      imu.mag_ready = false;
      imu.acc_ready = false;
      float res = rsp.heading - curr_ang;
      if (curr_ang > 90 and rsp.heading < -90){
        res = 360 + rsp.heading - curr_ang;
      }else if (curr_ang < -90 and rsp.heading > 90){
        res = -360 + rsp.heading - curr_ang;
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
    target_ang += side/1000.0;
  }

  if (target_ang > 180){
    target_ang-=360;
  }else if (target_ang < -180){
    target_ang+=360;
  }
  
  //motor1.write(fov-side);
  //motor2.write(fov+side);
  if (abs(ang_diff)>1){
    motor1.write(fov+min(max(-70, ang_diff*0.6+ang_int*0.5+delta_ang*2), 70));
    motor2.write(-fov+min(max(-70, ang_diff*0.6+ang_int*0.5+delta_ang*2), 70));
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