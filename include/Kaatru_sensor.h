#include <Wire.h>
#include <Arduino.h>
struct imu_data{
    int16_t acx;
    int16_t acy;
    int16_t acz;
    int16_t gcx;
    int16_t gcy;
    int16_t gcz;
    int16_t tmp;
    unsigned long millis = -1;
};
struct Stack {
    int top;
    unsigned capacity;
    imu_data* array;
};
void mpuInit(TwoWire* Wire);
int16_t getAccelx(TwoWire* Wire);
int16_t getAccely(TwoWire* Wire);
struct imu_data getMotion(TwoWire* Wire);
void getParticulateMatter(HardwareSerial* PM_Serial,int* PM01Value, int* PM2_5Value,int* PM10Value);
char checkValue(unsigned char *thebuf, char leng);
int transmitPM01(unsigned char *thebuf);
int transmitPM2_5(unsigned char *thebuf);
int transmitPM10(unsigned char *thebuf);
int analogSample(int number_of_samples, int delay_between_samples, int analogpin);
struct Stack* create_imu_stack(unsigned capacity);
void push(struct Stack* stack, imu_data item);
imu_data pop(struct Stack* stack);
int isEmpty(struct Stack* stack);
int isFull(struct Stack* stack);