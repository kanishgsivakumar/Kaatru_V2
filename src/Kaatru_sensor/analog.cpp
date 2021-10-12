#include <Arduino.h>

int analogSample(int number_of_samples, int delay_between_samples, int analogpin){
    int analogvalue = 0;
    for(int i = 0;i<number_of_samples;i++){
        analogvalue += analogRead(analogpin);
        delay(delay_between_samples);
    }
    return analogvalue/number_of_samples;

}