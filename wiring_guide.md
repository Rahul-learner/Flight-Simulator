# Complete Hardware Wiring Guide: Pico + Sensors + ESP8266

This document provides exact pinout mappings and connection diagrams for the 9-DOF Flight Attitude System with Raspberry Pi Pico, MPU6050, HMC5883L, BMP180, and ESP8266.

---

## 1. System Block Diagram

```
                             +------------------------+
                             |   PC (USB Debug/Dev)   |
                             +-----------+------------+
                                         | USB Cable (5V Power + Serial)
                                         v
                         +-------------------------------+
                         |      Raspberry Pi Pico        |
                         |        (RP2040 @ 250MHz)      |
                         |   9-DOF Multiplicative EKF    |
                         +---------------+---------------+
                                 |               |
              I2C0 Bus           |               | UART0 (115200 Baud)
   (GP20: SDA, GP21: SCL)        |               | (GP0: TX, GP1: RX)
   +--------------+--------------+               |
   |              |              |               v
   v              v              v       +---------------+
+-------+     +--------+     +--------+  |    ESP8266    |
|MPU6050|     |HMC5883L|     | BMP180 |  |  (NodeMCU/D1) |
| (IMU) |     |(Compass|     | (Baro) |  +-------+-------+
+-------+     +--------+     +--------+          |
(Addr: 0x68)  (0x1E/0x0D)    (Addr: 0x77)        | Wi-Fi AP (192.168.4.1)
                                                 v
                                        +-----------------+
                                        | Laptop / Phone  |
                                        | 3D Flight Deck  |
                                        +-----------------+
```

---

## 2. Complete Pin Mapping Table

| Device | Device Pin | Raspberry Pi Pico Pin | Pico Physical Pin # | Description |
| :--- | :--- | :--- | :--- | :--- |
| **MPU6050** | VCC | 3V3 (OUT) | Pin 36 | 3.3V Power |
| | GND | GND | Pin 38 | Common Ground |
| | SDA | GP20 (I2C0 SDA) | Pin 26 | I2C Data |
| | SCL | GP21 (I2C0 SCL) | Pin 27 | I2C Clock |
| | INT | GP15 | Pin 20 | 2000Hz Data Ready IRQ |
| | AD0 | GND | - | Address 0x68 |
| **HMC5883L** | VCC | 3V3 (OUT) | Pin 36 | 3.3V Power |
| | GND | GND | Pin 38 | Common Ground |
| | SDA | GP20 (I2C0 SDA) | Pin 26 | I2C Data (Shared) |
| | SCL | GP21 (I2C0 SCL) | Pin 27 | I2C Clock (Shared) |
| **BMP180** | VCC | 3V3 (OUT) | Pin 36 | 3.3V Power |
| | GND | GND | Pin 38 | Common Ground |
| | SDA | GP20 (I2C0 SDA) | Pin 26 | I2C Data (Shared) |
| | SCL | GP21 (I2C0 SCL) | Pin 27 | I2C Clock (Shared) |
| **ESP8266** | RX (GPIO3) | GP0 (UART0 TX) | Pin 1 | Telemetry stream from Pico |
| | TX (GPIO1) | GP1 (UART0 RX) | Pin 2 | Commands from ESP to Pico |
| | GND | GND | Pin 3 or Pin 8 | Common Ground (MANDATORY) |
| | VIN / 5V | VBUS (5V) | Pin 40 | 5V Power from Pico USB |

---

## 3. Wiring Details & Tips

### A. Shared I2C Bus (GP20 & GP21)
All three sensors communicate on the same physical I2C bus:
- Connect the **SDA** pin of MPU6050, HMC5883L, and BMP180 together to **GP20**.
- Connect the **SCL** pin of MPU6050, HMC5883L, and BMP180 together to **GP21**.
- Connect all sensor **VCC** pins to Pico **3V3 (Pin 36)**.
- Connect all sensor **GND** pins to a Pico **GND (Pin 38)**.
- *I2C Addresses*:
  - MPU6050 = `0x68`
  - HMC5883L = `0x1E` (or QMC5883L = `0x0D`, both auto-detected)
  - BMP180 = `0x77`
  - No address conflicts exist!

### B. UART Bridge to ESP8266
- **Pico GP0 (TX)** connects to **ESP8266 RX (GPIO3)**.
- **Pico GP1 (RX)** connects to **ESP8266 TX (GPIO1)** (allows web browser to send `TARE` and `CALIB` back to the Pico).
- **Common Ground**: Ensure a solid connection between Pico GND and ESP8266 GND. Without common ground, serial signals float and packets get corrupted.
- **Powering ESP8266**: The ESP8266 requires around 80–170 mA during Wi-Fi transmission. You can power it by connecting the Pico's `VBUS` (Pin 40, 5V from PC USB) to the `VIN` or `5V` pin on the NodeMCU / Wemos D1 Mini.

### C. MPU6050 AUX Bypass Feature
In `Koyal_Flight_Controller.cpp`, the MPU6050 is configured with:
```cpp
i2c_write_byte(MPU6050_ADDR, MPU6050_INT_PIN_CFG, 0x02); // BYPASS_EN = 1
```
This enables the internal pass-through multiplexer. Even if your HMC5883L or BMP180 is connected to the MPU6050's `AUX_DA` and `AUX_CL` header pins, they are passed directly through to the Pico's GP20/GP21 bus!

---

## 4. Verification Check
When you flash the Pico and open the PC Serial Monitor at 115200 baud, you will see an automated I2C scan:
```text
[INIT] Scanning I2C bus for connected sensors...
  -> Found I2C Device at 0x0D (QMC5883L Compass)   [or 0x1E for HMC5883L]
  -> Found I2C Device at 0x68 (MPU6050 IMU)
  -> Found I2C Device at 0x77 (BMP180 Barometer)
```
If all 3 devices appear, the bus is wired correctly and the 9-DOF AHRS starts streaming.
