
 //******************************
 //*Abstract: Read value of PM1,PM2.5 and PM10 of air quality
 //
 //******************************
#include <Arduino.h>
#define LENG 31   //0x42 + 31 bytes equal to 32 bytes
unsigned char buf[LENG];

char checkValue(unsigned char *thebuf, char leng)
{
  char receiveflag=0;
  int receiveSum=0;

  for(int i=0; i<(leng-2); i++){
  receiveSum=receiveSum+thebuf[i];
  }
  receiveSum=receiveSum + 0x42;

  if(receiveSum == ((thebuf[leng-2]<<8)+thebuf[leng-1]))  //check the serial data
  {
    receiveSum = 0;
    receiveflag = 1;
  }
  
  return receiveflag;
}

int transmitPM01(unsigned char *thebuf)
{
  int PM01Val;
  PM01Val=((thebuf[3]<<8) + thebuf[4]); //count PM1.0 value of the air detector module
  return PM01Val;
}


int transmitPM2_5(unsigned char *thebuf)
{
  int PM2_5Val;
  PM2_5Val=((thebuf[5]<<8) + thebuf[6]);//count PM2.5 value of the air detector module
  return PM2_5Val;
  }


int transmitPM10(unsigned char *thebuf)
{
  int PM10Val;
  PM10Val=((thebuf[7]<<8) + thebuf[8]); //count PM10 value of the air detector module
  return PM10Val;
}

void getParticulateMatter(HardwareSerial* PM_Serial,int* PM01Value, int* PM2_5Value,int* PM10Value)
{
    //Serial.println("PM_START");
  if(PM_Serial->find(0x42)){    //start to read when detect 0x42
    Serial.print("Fetch");
    PM_Serial->readBytes(buf,LENG);
    if(buf[0] == 0x4d){
      if(checkValue(buf,LENG)){
        
        *PM01Value=transmitPM01(buf); //count PM1.0 value of the air detector module
        *PM2_5Value=transmitPM2_5(buf);//count PM2.5 value of the air detector module
        *PM10Value=transmitPM10(buf); //count PM10 value of the air detector module
      }
    }
  }



}
