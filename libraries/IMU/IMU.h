#include <Wire.h>
#include <Arduino.h>
#include <EEPROM.h>
//#include <typeinfo>

#define acc_address 0b0011001
#define mag_address 0b0011110


class IMU{
    template <typename T> struct vector
    {
        T x, y, z;
    };
public:
    uint8_t AM_stat = 0;
    vector<int16_t> a; // accelerometer readings
    vector<int16_t> m; // magnetometer readings
    vector<int16_t> m_min =  {  -610,   -713,    -25};
    vector<int16_t> m_max = {  +459,   +341,    +10};
    byte wire_inp[6];
    uint8_t winp_c = 0;
    bool listen_wire = true;
    bool last_status = false;
    unsigned long time = 0;


public:
    float heading_res = 0;

    void init();
    void writeAccReg(byte reg, byte value);
    void writeMagReg(byte reg, byte value);
    bool writeReg(uint8_t address, uint8_t reg);
    void requestAcc();
    void readAcc();
    void requestMag();
    void readMag();
    void update();
    void calibrate();
    void vector_normalize(vector<float> *a)
    {
        float mag = sqrt(vector_dot(a, a));
        a->x /= mag;
        a->y /= mag;
        a->z /= mag;
    }

    float heading(vector<int> from);

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

};