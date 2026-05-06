#include <Arduino.h>
#include <ArduinoJson.h>
#include "jetson_link.h"
#include "robot_drive.h"
#include "robot_imu.h"
#include "robot_motion_control.h"
#include "wireless.h"   // robotMessage (odom), controllerMessage (joystick)

// ---- Globals exposed to followTrajectory() ----
volatile double   g_jetson_vel_v   = 0.0;
volatile double   g_jetson_vel_w   = 0.0;
volatile uint32_t g_jetson_vel_ts_ms = 0;
volatile bool     g_jetson_estop   = false;

// Read by sendJetsonTelemetry() to compute vm/wm from measured wheel speeds.
// Declared in robot_drive.cpp (file-scope double[4]); we re-declare extern here.
extern double velocities[];

// Line accumulator: Serial reads can deliver partial lines; we accrue until '\n'.
static String rx_buf;

static void handleVelCmd(JsonDocument& doc) {
    g_jetson_vel_v     = doc["v"] | 0.0;
    g_jetson_vel_w     = doc["w"] | 0.0;
    g_jetson_vel_ts_ms = millis();
    g_jetson_estop     = false;
}

static void handleEstopCmd() {
    g_jetson_estop     = true;
    g_jetson_vel_v     = 0.0;
    g_jetson_vel_w     = 0.0;
    g_jetson_vel_ts_ms = 0;
}

static void dispatch(const String& line) {
    // StaticJsonDocument<256> doc;
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, line);
    if (err) return;  // silent drop — matches Jetson-side policy

    const char* cmd = doc["cmd"] | "";
    if      (!strcmp(cmd, "vel"))     handleVelCmd(doc);
    else if (!strcmp(cmd, "estop"))   handleEstopCmd();
    // arm / gripper / ramp: accept-and-ignore for this round (TODO)
}

void setupJetsonLink() {
    rx_buf.reserve(256);
    Serial.println("{\"status\":\"ready\",\"device\":\"ESP32-S3\"}");
}

void handleJetsonSerial() {
    // Must be called every loop iteration — at 921600 baud the USB-CDC
    // ring fills fast; gating behind EVERY_N_MILLIS drops bytes mid-packet.
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\n') {
            rx_buf.trim();
            if (rx_buf.length() > 0) dispatch(rx_buf);
            rx_buf = "";
        } else if (c != '\r') {
            rx_buf += c;
            if (rx_buf.length() > 240) rx_buf = "";  // runaway guard
        }
    }
}

void sendJetsonTelemetry() {
    // Indices 1 (right wheel PID) and 2 (left wheel PID) — see robot_drive.cpp.
    double velR = velocities[1];  // rad/s
    double velL = velocities[2];
    double vm = 0.5 * R_WHEEL * (velL + velR);
    double wm = R_WHEEL * (velR - velL) / (2.0 * B_BASE);

    uint32_t now = millis();
    uint32_t age = now - g_jetson_vel_ts_ms;
    uint8_t flags = 0;
    if (g_jetson_estop)                                      flags |= 0x01;
    if (g_jetson_vel_ts_ms == 0 || age >= 200)               flags |= 0x02;


    // StaticJsonDocument<256> doc;
    JsonDocument doc;
    doc["t"]     = now;
    doc["x"]     = robotMessage.x;
    doc["y"]     = robotMessage.y;
    doc["th"]    = robotMessage.theta;
    doc["vm"]    = vm;
    doc["wm"]    = wm;
    doc["roll"]  = ypr.roll;
    doc["flags"] = flags;

    serializeJson(doc, Serial);
    Serial.write('\n');
}
