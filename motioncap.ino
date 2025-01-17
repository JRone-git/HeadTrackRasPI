#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "Adafruit_TinyUSB.h"

uint8_t const desc_hid_report[] = {
    TUD_HID_REPORT_DESC_GAMEPAD()
};

Adafruit_USBD_HID usb_hid;
Adafruit_MPU6050 mpu;

const int LED_PIN = 25;
const int BUTTON_PIN = 15;
const float SENSITIVITY = 100.0;  // Scaling factor for joystick values
const float DEADZONE = 0.5;       // Minimal gyro movement to register
const float SMOOTHING = 0.95;     // Low-pass filter factor

float yaw = 0, pitch = 0, roll = 0; // Cumulative angles
unsigned long lastTime = 0;

void setup() {
    Serial.begin(115200);

    pinMode(LED_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Set button pin as input with pull-up resistor
    digitalWrite(LED_PIN, HIGH);

    usb_hid.setPollInterval(2);
    usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
    usb_hid.begin();

    while (!TinyUSBDevice.mounted()) delay(1);

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

void resetOrientation() {
    yaw = 0;
    pitch = 0;
    roll = 0;
}

void sendJoystickReport(float x, float y, float z) {
    // Apply deadzone and map to joystick range (-127 to 127)
    int8_t mappedX = (abs(x) > DEADZONE) ? (int8_t)(constrain(x * SENSITIVITY, -250, 250) * 127.0 / 250.0) : 0;
    int8_t mappedY = (abs(y) > DEADZONE) ? (int8_t)(constrain(y * SENSITIVITY, -250, 250) * 127.0 / 250.0) : 0;
    int8_t mappedZ = (abs(z) > DEADZONE) ? (int8_t)(constrain(z * SENSITIVITY, -250, 250) * 127.0 / 250.0) : 0;

    // Debug output
    Serial.print("Pitch: "); Serial.print(x);
    Serial.print(" | Roll: "); Serial.print(y);
    Serial.print(" | Yaw: "); Serial.println(z);

    hid_gamepad_report_t report;
    memset(&report, 0, sizeof(report));
    report.x = mappedX;
    report.y = mappedY;
    report.z = mappedZ;

    if (usb_hid.ready()) {
        usb_hid.sendReport(0, &report, sizeof(report));
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
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

    // Keep cumulative angles in a sensible range (-180 to 180 degrees)
    if (yaw > 180) yaw -= 360;
    else if (yaw < -180) yaw += 360;

    // Check for button press to reset orientation
    if (digitalRead(BUTTON_PIN) == LOW) {
        delay(50); // Debounce delay
        if (digitalRead(BUTTON_PIN) == LOW) {
            resetOrientation();
            while (digitalRead(BUTTON_PIN) == LOW); // Wait for button release
        }
    }

    sendJoystickReport(smoothedPitch, smoothedRoll, smoothedYaw);
    delay(10); // Polling interval
}
