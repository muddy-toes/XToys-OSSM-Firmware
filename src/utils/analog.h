#ifndef OSSM_SOFTWARE_ANALOG_H
#define OSSM_SOFTWARE_ANALOG_H

#include "Arduino.h"

typedef struct {
    int pinNumber;
    int samples;
} SampleOnPin;

// public static function to get the analog value of a pin
static float getAnalogAveragePercent(int pinNumber, int samples) {
    int sum = 0;
    float average;
    float percentage;

    for (int i = 0; i < samples; i++) {
        // TODO: Possibly use fancier filters?
        sum += analogRead(pinNumber);
    }
    average = (float)sum / (float)samples;
    // TODO: Might want to add a dead-band
    percentage = 100.0f * average / 4096.0f;  // 12 bit resolution
    return percentage;
}

#endif  // OSSM_SOFTWARE_ANALOG_H