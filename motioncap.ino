#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Adafruit_TinyUSB.h"

uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_GAMEPAD()
};

Adafruit_USBD_HID usb_hid;
Adafruit_MPU6050 mpu;

// Axes
float yaw = 0;
float pitch = 0;
float roll = 0;

// Smoothing
const float SMOOTHING = 0.9;

// Pins
const int LED_PIN = 25;
const int CENTER_BUTTON = 15;

// Timing
unsigned long lastTime = 0;

void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    pinMode(CENTER_BUTTON, INPUT_PULLUP); // Use internal pull-up resistor
    digitalWrite(LED_PIN, HIGH);

    // USB HID setup
    usb_hid.setPollInterval(2);
    usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
    usb_hid.begin();

    while (!TinyUSBDevice.mounted()) delay(1);

    // MPU6050 setup
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();

    if (!mpu.begin()) {
        while (1) {
            digitalWrite(LED_PIN, HIGH);
            delay(100);
            digitalWrite(LED_PIN, LOW);
            delay(100);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    digitalWrite(LED_PIN, LOW);
    lastTime = millis();
}

void sendJoystickReport(float pitch, float roll, float yaw) {
    // Map to int8_t range (-128 to 127)
    int8_t mappedPitch = (int8_t)(constrain(pitch, -90, 90) * 127.0 / 90.0);
    int8_t mappedRoll = (int8_t)(constrain(roll, -90, 90) * 127.0 / 90.0);
    int8_t mappedYaw = (int8_t)(constrain(yaw, -180, 180) * 127.0 / 180.0);

    // Apply deadzone
    const int8_t DEADZONE = 5;
    if (abs(mappedPitch) < DEADZONE) mappedPitch = 0;
    if (abs(mappedRoll) < DEADZONE) mappedRoll = 0;
    if (abs(mappedYaw) < DEADZONE) mappedYaw = 0;

    hid_gamepad_report_t report;
    memset(&report, 0, sizeof(report));
    report.x = mappedYaw;
    report.y = mappedPitch;
    report.z = mappedRoll;

    if (usb_hid.ready()) {
        usb_hid.sendReport(0, &report, sizeof(report));
        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED
    }
}

void loop() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Delta time in seconds
    lastTime = currentTime;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Integrate gyroscope data to calculate cumulative angles
    pitch += g.gyro.x * dt;
    roll += g.gyro.y * dt;
    yaw += g.gyro.z * dt;

    // Apply smoothing filter
    static float smoothedPitch = 0, smoothedRoll = 0, smoothedYaw = 0;
    smoothedPitch = SMOOTHING * smoothedPitch + (1 - SMOOTHING) * pitch;
    smoothedRoll = SMOOTHING * smoothedRoll + (1 - SMOOTHING) * roll;
    smoothedYaw = SMOOTHING * smoothedYaw + (1 - SMOOTHING) * yaw;

    // Center angles if button is pressed
    if (digitalRead(CENTER_BUTTON) == LOW) {
        delay(50); // Debounce
        if (digitalRead(CENTER_BUTTON) == LOW) {
            smoothedPitch = 0;
            smoothedRoll = 0;
            smoothedYaw = 0;
            pitch = 0;
            roll = 0;
            yaw = 0;
            while (digitalRead(CENTER_BUTTON) == LOW) delay(10); // Wait for button release
        }
    }

    // Keep yaw in -180 to 180 range
    if (yaw > 180) yaw -= 360;
    else if (yaw < -180) yaw += 360;

    sendJoystickReport(smoothedPitch, smoothedRoll, smoothedYaw);
    delay(10); // Polling interval
}
