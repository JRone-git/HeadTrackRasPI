

# **MPU6050 HeadTracker with Raspberry Pi Pico**

This project transforms a Raspberry Pi Pico with an MPU6050 sensor into a headtracker by simulating a joystick for games. The device uses the MPU6050’s gyroscope and accelerometer to control yaw, pitch, and roll axes. It also includes a button to center the axes.

---

## **Features**
- **Yaw, Pitch, Roll Mapping**: Smooth and precise axis mapping for game controllers.
- **Centering Button**: A dedicated button to reset all axes to the center.
- **Plug and Play**: Works as a USB HID game controller.

---

## **Requirements**
1. Raspberry Pi Pico
2. MPU6050 sensor
3. Push button (for centering)
4. Wires and breadboard (optional)

---

## **Pinout and Wiring**

| **Raspberry Pi Pico Pin** | **MPU6050 Pin**     | **Additional Component**          |
|----------------------------|---------------------|------------------------------------|
| **GP4 (SDA)**             | SDA                 |                                    |
| **GP5 (SCL)**             | SCL                 |                                    |
| **3.3V**                  | VCC                 |                                    |
| **GND**                   | GND                 |                                    |
| **GP15**                  | Button (One terminal connected to GP15, the other to GND) |

### **Schematic**
- MPU6050:
  - **VCC** to Pico **3.3V**
  - **GND** to Pico **GND**
  - **SDA** to Pico **GP4**
  - **SCL** to Pico **GP5**

- Centering Button:
  - One terminal of the button to **GP15**.
  - The other terminal of the button to **GND**.

---

## **Setup and Usage**

### **1. Install Required Libraries**
- Install the **Adafruit_TinyUSB** and **Adafruit_MPU6050** libraries in your Arduino IDE.

### **2. Flash the Code**
- Connect the Pico in bootloader mode:
  1. Hold the BOOTSEL button while plugging the Pico into your PC.
  2. Drag and drop the Pico's UF2 file (or upload via Arduino IDE).

### **3. Wiring**
- Use the schematic above to connect the MPU6050 and button to the Raspberry Pi Pico.

### **4. Verify Functionality**
- Open the **Game Controllers** window in Windows:
  - Search for `Game Controllers` in the Start menu.
  - Select your device from the list.
  - Observe movement of the axes as you tilt and turn the MPU6050.

### **5. Using the Centering Button**
- Press the button to reset all axes to the center position.

---

## **Troubleshooting**

1. **No movement in the Game Controllers window?**
   - Check the connections for SDA and SCL.
   - Ensure the MPU6050 is powered properly.

2. **Axes snapping or unstable?**
   - Verify that the MPU6050 is mounted securely.
   - Ensure no external vibrations interfere with the readings.

3. **No USB device detected?**
   - Confirm that TinyUSB libraries are installed and functioning.

---

## **Diagram**

Here’s a visual representation of the wiring:

```plaintext
   Raspberry Pi Pico                 MPU6050
      (3.3V) ------------------------ VCC
       (GND) ------------------------ GND
       (GP4) ------------------------ SDA
       (GP5) ------------------------ SCL

   Raspberry Pi Pico                 Button
      (GP15) ---------------- One terminal
       (GND) ---------------- Other terminal
```

