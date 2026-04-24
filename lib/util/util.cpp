#include "util.h"
#include <Arduino.h>

//map from an input to output range linearly
double mapDouble(double x, double in_min, double in_max, double out_min, double out_max) {
  //if (x <= in_min) return out_min;
  //if (x >= in_max) return out_max;
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void printTabs(uint8_t nTabs) {
    for (uint8_t i = 0; i < nTabs; ++i) {
      if (Serial)
        Serial.print("\t");
    }
}

double mapStick(double v, double MAG) {
    // dead zone
    double dead = 0.1;
    if (fabs(v) < dead) return 0;

    if (v > 0) {
        return mapDouble(v, dead, 1.0, 0, MAG);
    } else {
        return mapDouble(v, -1.0, -dead, -MAG, 0);
    }
}
