#ifndef JETSON_LINK_H
#define JETSON_LINK_H

#include <Arduino.h>

// Latest Jetson velocity command, set by handleJetsonSerial().
// g_jetson_vel_ts_ms == 0 sentinels "no command ever received".
extern volatile double   g_jetson_vel_v;        // m/s
extern volatile double   g_jetson_vel_w;        // rad/s
extern volatile uint32_t g_jetson_vel_ts_ms;
extern volatile bool     g_jetson_estop;

void setupJetsonLink();      // prints {"status":"ready"} once
void handleJetsonSerial();   // non-blocking; call every loop() iteration
void sendJetsonTelemetry();  // call at 50 Hz

#endif // JETSON_LINK_H
