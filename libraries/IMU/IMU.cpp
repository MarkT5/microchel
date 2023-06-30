#include "IMU.h"


void IMU::init(){
    Wire.begin();
    EEPROM.begin();
    EEPROM.get(0, m_min);
    EEPROM.get(sizeof(m_min)+1, m_max);
    writeAccReg(0x23, 0x08);
    writeAccReg(0x20, 0x47);
    writeMagReg(0x00, 0x0C);
    writeMagReg(0x01, 0x20);
    writeMagReg(0x02, 0x00);
}

void IMU::writeAccReg(byte reg, byte value)
{
    Wire.beginTransmission(acc_address);
    Wire.write(reg);
    Wire.write(value);
    last_status = Wire.endTransmission();
}

void IMU::writeMagReg(byte reg, byte value)
{
    Wire.beginTransmission(mag_address);
    Wire.write(reg);
    Wire.write(value);
    last_status = Wire.endTransmission();
}

void IMU::update() {
    unsigned long curr_time = millis();
    heading_res = heading((vector<int>){0, -1, 0});
    if (winp_c < 6 && listen_wire && Wire.available()){
        wire_inp[winp_c] = Wire.read();
        winp_c+=1;
    }
    if (AM_stat == 0 || curr_time-time > 5000){
        time = curr_time;
        winp_c = 0;
        while (Wire.available()){
            Wire.read();
        }
        requestAcc();
        AM_stat = 1;
    }else if(AM_stat == 1 && winp_c == 6){
        readAcc();
        requestMag();
        AM_stat = 2;
    }else if(AM_stat == 2 && winp_c == 6){
        readMag();
        AM_stat = 0;
    }

}

// IMU

bool IMU::writeReg(uint8_t address, uint8_t reg)
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



void IMU::requestAcc()
{
    Wire.beginTransmission(acc_address);
    Wire.write(0x28 | (1 << 7));
    last_status = Wire.endTransmission();
    Wire.requestFrom((uint8_t)acc_address, (uint8_t)6);
    listen_wire = true;
    winp_c = 0;
}

void IMU::readAcc(){
    a.x = (int16_t)(wire_inp[1] << 8 | wire_inp[0]);
    a.y = (int16_t)(wire_inp[3] << 8 | wire_inp[2]);
    a.z = (int16_t)(wire_inp[5] << 8 | wire_inp[4]);
    winp_c = 0;
    listen_wire = false;
}

void IMU::requestMag()
{
    Wire.beginTransmission(mag_address);
    Wire.write(0x03);
    last_status = Wire.endTransmission();
    Wire.requestFrom((uint8_t)mag_address, (byte)6);
    listen_wire = true;
    winp_c = 0;
}

void IMU::readMag(){
    m.x = (int16_t)(wire_inp[0] << 8 | wire_inp[1]);
    m.y = (int16_t)(wire_inp[4] << 8 | wire_inp[5]);
    m.z = (int16_t)(wire_inp[2] << 8 | wire_inp[3]);
    winp_c = 0;
    listen_wire = false;
}


float IMU::heading(vector<int> from)
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


void IMU::calibrate() {
    vector <int16_t> running_min = {32767, 32767, 32767}, running_max = {-32768, -32768, -32768};

    for (int i = 0; i < 10000; i++) {
        update();
        running_min.x = min(running_min.x, m.x);
        running_min.y = min(running_min.y, m.y);
        running_min.z = min(running_min.z, m.z);

        running_max.x = max(running_max.x, m.x);
        running_max.y = max(running_max.y, m.y);
        running_max.z = max(running_max.z, m.z);
    }

    m_min.x = running_min.x;
    m_min.y = running_min.y;
    m_min.z = running_min.z;
    m_max.x = running_max.x;
    m_max.y = running_max.y;
    m_max.z = running_max.z;
    EEPROM.put(0, m_min);
    EEPROM.put(sizeof(m_min)+1, m_max);
}