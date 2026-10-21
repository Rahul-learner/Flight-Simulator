# Koyal Flight Controller: Advanced 9-DOF AHRS & 3D Flight Deck Simulator

An advanced aerospace-grade **9-DOF Attitude and Heading Reference System (AHRS)** with **Barometric Altimeter** for Raspberry Pi Pico (RP2040) paired with an **ESP8266 Wi-Fi Telemetry Server** and an offline-first **Interactive 3D WebGL Flight Deck**.

---

## 🌟 System Overview & Key Upgrades

### 1. Multiplicative Extended Kalman Filter (MEKF / Error-State EKF)
The original 7-state quaternion EKF in `Koyal_Flight_Controller.cpp` suffered from a fundamental numerical singularity: unconstrained 4D quaternion covariance updates frequently degraded positive semi-definiteness, causing the innovation covariance matrix $S$ inversion to fail (`"Error: S matrix is singular"`).

The new [`AdvancedAttitudeEKF.hpp`](file:///mnt/Vol_B/HobbyProject/Flight%20Simulator/AdvancedAttitudeEKF.hpp) implements an aerospace-grade **6-DOF Error-State Extended Kalman Filter**:
- **Minimal Error-State Representation ($\delta \mathbf{x} \in \mathbb{R}^6$)**:
  $\delta \mathbf{x} = [\delta \boldsymbol{\theta}^T, \delta \mathbf{b}_g^T]^T$ representing small 3-axis rotation angle errors and 3-axis gyroscope bias errors. This completely eliminates quaternion covariance singularity.
- **Closed-Form Rodrigues Kinematics**:
  Quaternion propagation uses exact exponential map rotation integration over time step $\Delta t$, eliminating first-order Euler integration truncation errors during aggressive turns.
- **Joseph-Form Stabilized Covariance Update**:
  $$P_{k} = (\mathbf{I} - \mathbf{K}\mathbf{H}) P_{k}^- (\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K} \mathbf{R} \mathbf{K}^T$$
  Guarantees that covariance $P$ remains symmetric and positive semi-definite under all conditions.
- **Dynamic Acceleration Gating & Adaptive Noise**:
  When centrifugal forces or linear acceleration occur ($|\|\mathbf{a}\| - 1.0g| > 0.40g$), the filter adaptively downweights or inhibits accelerometer updates, preventing the artificial horizon from falsely tilting in turns.
- **9-DOF Magnetic Heading Fusion**:
  Fuses horizontal components of the HMC5883L / QMC5883L magnetometer, locking Yaw directly to Magnetic North and eliminating yaw drift forever.

### 2. Sensor Suite Drivers
- [`MPU6050`](file:///mnt/Vol_B/HobbyProject/Flight%20Simulator/Koyal_Flight_Controller.cpp): 6-DOF IMU operating at 2000 Hz with falling-edge interrupt on GP15 and automatic I2C bypass mode.
- [`HMC5883L.hpp`](file:///mnt/Vol_B/HobbyProject/Flight%20Simulator/HMC5883L.hpp): 3-Axis Digital Compass with auto-detection for both genuine Honeywell HMC5883L (`0x1E`) and QST QMC5883L (`0x0D`) chips.
- [`BMP180.hpp`](file:///mnt/Vol_B/HobbyProject/Flight%20Simulator/BMP180.hpp): Bosch Barometric Pressure & Temperature sensor with asynchronous non-blocking state machine (zero jitter for 2000 Hz loop), relative altitude AGL calculation, and variometer (vertical speed climb rate in m/s).

### 3. ESP8266 Webserver & 3D WebGL Flight Deck
- **Hardware UART Bridge**: Raspberry Pi Pico streams JSON telemetry packets over hardware UART (`uart0` on GP0/GP1 at 115200 baud) directly to the ESP8266.
- **Standalone Wi-Fi Hotspot**: ESP8266 creates an Access Point (`PicoFlight-Sim`, IP: `192.168.4.1`) or connects to local Wi-Fi.
- **Zero-Dependency 3D Aircraft Model**:
  - Procedural aerodynamic aircraft (fuselage, wings, ailerons, vertical fin, elevators, cockpit canopy, wingtip navigation lights, and spinning propeller).
  - Rotates in real-time according to the Pico's unit quaternion / Euler angles.
  - Translates vertically in 3D perspective based on the BMP180 barometric altitude!
  - 100% self-contained in Flash PROGMEM (`web_assets.h`): works completely offline without requiring any external internet or CDN downloads!
  - Integrated Attitude Director Indicator (ADI / Artificial Horizon), Compass Rose Heading Tape, and live sensor telemetry HUD.

---

## 📁 Project Directory Structure

```
.
├── AdvancedAttitudeEKF.hpp        # 6-DOF Error-State Quaternion Kalman Filter
├── HMC5883L.hpp                  # Auto-detecting HMC5883L / QMC5883L I2C driver
├── BMP180.hpp                    # Non-blocking BMP180 Barometer & Variometer driver
├── Koyal_Flight_Controller.cpp    # Main Pico Flight Controller firmware
├── CMakeLists.txt                 # Pico SDK CMake build configuration
├── pico_sdk_import.cmake          # Pico SDK import helper
├── wiring_guide.md                # Pinout mappings and connection diagrams
├── ESP8266_Flight_Server/
│   ├── ESP8266_Flight_Server.ino  # ESP8266 Arduino sketch (Webserver + WebSockets)
│   ├── web_assets.h               # Embedded 3D flight deck web app (PROGMEM)
│   ├── index.html                 # Standalone HTML5/WebGL 3D flight simulator
│   └── generate_header.py         # Utility to pack index.html into web_assets.h
└── README.md                      # Complete system documentation
```

---

## ⚡ Hardware Pinout Summary

All sensors share the Pico **I2C0** bus:
- **GP20 (Pin 26)**: Shared I2C0 SDA (MPU6050, HMC5883L, BMP180)
- **GP21 (Pin 27)**: Shared I2C0 SCL (MPU6050, HMC5883L, BMP180)
- **GP15 (Pin 20)**: MPU6050 Data Ready Interrupt (active low)
- **3V3 (Pin 36)**: 3.3V power to all three sensor breakout boards
- **GND (Pin 38)**: Common Ground

UART connection between Pico and ESP8266:
- **Pico GP0 (Pin 1, TX)** -> **ESP8266 RX (GPIO3)**
- **Pico GP1 (Pin 2, RX)** <- **ESP8266 TX (GPIO1)**
- **Pico GND** -> **ESP8266 GND** (Mandatory common ground)
- **Pico VBUS (Pin 40, 5V)** -> **ESP8266 VIN / 5V**

*(For full wiring diagrams, see [`wiring_guide.md`](file:///mnt/Vol_B/HobbyProject/Flight%20Simulator/wiring_guide.md).)*

---

## 🚀 Quickstart & Flashing Instructions

### 1. Build and Flash Raspberry Pi Pico
Prerequisites: Raspberry Pi Pico SDK (`pico-sdk`).

```bash
# 1. Create build directory
mkdir build && cd build

# 2. Configure with CMake
cmake .. -DPICO_SDK_PATH=/path/to/pico-sdk

# 3. Compile
make -j$(nproc)
```
Hold the `BOOTSEL` button on your Raspberry Pi Pico, plug it into your PC USB, and drag-and-drop the generated `Koyal_Flight_Controller.uf2` file onto the `RPI-RP2` drive.

### 2. Flash the ESP8266
1. Open the Arduino IDE.
2. Go to **Library Manager** (`Ctrl+Shift+I`) and install:
   - `WebSockets` by Markus Sattler
3. Open [`ESP8266_Flight_Server/ESP8266_Flight_Server.ino`](file:///mnt/Vol_B/HobbyProject/Flight%20Simulator/ESP8266_Flight_Server/ESP8266_Flight_Server.ino).
4. Select board: `NodeMCU 1.0 (ESP-12E Module)` or `LOLIN(WEMOS) D1 R2 & mini`.
5. Connect ESP8266 via USB and click **Upload**.

### 3. Open the 3D Flight Simulator
1. Power on the Pico and ESP8266.
2. Keep the sensor stationary on a table for 2 seconds while the Pico calibrates and locks to Magnetic North.
3. On your laptop, phone, or tablet, connect to the Wi-Fi network:
   - **SSID**: `PicoFlight-Sim`
   - **Password**: `flightattitude`
4. Open your web browser and navigate to:
   **`http://192.168.4.1`**
5. The 3D airplane will immediately appear and track your sensor's Pitch, Roll, Yaw, and Barometric Altitude in real-time!

---

## 🧪 Testing Without Hardware (Browser Simulation)
You can test the 3D Flight Deck right now in your PC browser:
1. Open [`ESP8266_Flight_Server/index.html`](file:///mnt/Vol_B/HobbyProject/Flight%20Simulator/ESP8266_Flight_Server/index.html) in Chrome, Firefox, Edge, or Safari.
2. Check the **"Demo Flight Sim"** toggle switch on the right sidebar.
3. The interactive 3D airplane will simulate realistic roll, pitch, and yaw flight dynamics with live instruments!
