#include <Wire.h>
#include <SPI.h>
#include <RF24.h>

template <typename T> struct vector
{
  T x, y, z;
};

byte acc_address = 0b0011001;
byte mag_address = 0b0011110;
uint8_t AM_stat = 0;

vector<int16_t> a; // accelerometer readings
vector<int16_t> m; // magnetometer readings
vector<int16_t> m_min =  {  -610,   -713,    -25};
vector<int16_t> m_max = {  +459,   +341,    +10};



void initIMU(){
  writeAccReg(0x23, 0x08);
  writeAccReg(0x20, 0x47);
  writeMagReg(0x00, 0x0C);
  writeMagReg(0x01, 0x20);
  writeMagReg(0x02, 0x00);
}

void writeAccReg(byte reg, byte value)
{
  Wire.beginTransmission(acc_address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void writeMagReg(byte reg, byte value)
{
  Wire.beginTransmission(mag_address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}



void setup() {
  Serial.begin(115200);
  Wire.begin();
  initIMU();
}


unsigned int counter = 0;
unsigned int print_counter = 0;

byte wire_inp[6];
uint8_t winp_c = 0;
bool listen_wire = true;
float heading_res = 0;

void loop() {
  heading_res = heading((vector<int>){0, -1, 0});
  counter+=1;
  if (winp_c < 6 && listen_wire && Wire.available()){
    wire_inp[winp_c] = Wire.read();
    winp_c+=1;
  }
  if (AM_stat == 0 || counter > 2000){
    counter = 0;
    winp_c = 0;
    //Serial.print(AM_stat);
    //Serial.println("dumping");
    if (Wire.available()) Serial.println("dumping");
    while (Wire.available()) Wire.read();
    requestAcc();
    AM_stat = 1;
  }else if(AM_stat == 1 && winp_c == 6){
    readAcc();
    requestMag();
    AM_stat = 2;
  }else if(AM_stat == 2 && winp_c == 6){
    //Serial.print(AM_stat);
    readMag();
    AM_stat = 0;
  }
      
}

// IMU

bool writeReg(byte address, byte reg)
{
  Wire.beginTransmission(address);
  Wire.write((byte)reg);
  if (Wire.endTransmission() != 0)
  {
    return false;
  }

  Wire.requestFrom(address, (byte)1);
  if (Wire.available())
  {
    return Wire.read();
  }
  else
  {
    return false;
  }
}



void requestAcc(void)
{
  Wire.beginTransmission(acc_address);
  Wire.write(0x28 | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(acc_address, (byte)6);
  listen_wire = true;
  winp_c = 0;
}

void readAcc(){
  a.x = (int16_t)(wire_inp[1] << 8 | wire_inp[0]);
  a.y = (int16_t)(wire_inp[3] << 8 | wire_inp[2]);
  a.z = (int16_t)(wire_inp[5] << 8 | wire_inp[4]);
  winp_c = 0;
  listen_wire = false;
}

void requestMag(void)
{
  Wire.beginTransmission(mag_address);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(mag_address, (byte)6);
  listen_wire = true;
  winp_c = 0;
}

void readMag(){
  m.x = (int16_t)(wire_inp[0] << 8 | wire_inp[1]);
  m.y = (int16_t)(wire_inp[4] << 8 | wire_inp[5]);
  m.z = (int16_t)(wire_inp[2] << 8 | wire_inp[3]);
  winp_c = 0;
  listen_wire = false;
}




// vectors


void vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}

float heading(vector<int> from)
{
    vector<int32_t> temp_m = {m.x, m.y, m.z};

    // subtract offset (average of min and max) from magnetometer readings
    temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
    temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
    temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;

    // compute E and N
    vector<float> E;
    vector<float> N;
    vector_cross(&temp_m, &a, &E);
    vector_normalize(&E);
    vector_cross(&a, &E, &N);
    vector_normalize(&N);

    // compute heading
    float heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / PI;
    if (heading < 0) heading += 360;
    return heading;
}

template <typename Ta, typename Tb, typename To> void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
{
  out->x = (a->y * b->z) - (a->z * b->y);
  out->y = (a->z * b->x) - (a->x * b->z);
  out->z = (a->x * b->y) - (a->y * b->x);
}

template <typename Ta, typename Tb> float vector_dot(const vector<Ta> *a, const vector<Tb> *b)
{
  return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
}