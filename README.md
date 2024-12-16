# Microproc.-Sys.-Based-Design
ECEN 260
# Multi-Sensor Environmental Monitor

## Project Overview
The Multi-Sensor Environmental Monitor is a portable system designed to measure and display environmental parameters in real-time. Using an STM32 Nucleo L476RG microcontroller, the device integrates multiple sensors and a LCD to provide a user-friendly dashboard for data visualization. The system monitors temperature, humidity, pressure, light intensity, air quality, and magnetic field strength, making it suitable for a variety of applications, including environmental monitoring, smart homes, and IoT systems.

---

## Features
- **Multi-Sensor Integration:**
  - **BME280**: Temperature, humidity, pressure, and altitude measurements (I2C communication).
  - **BH1750**: Light intensity readings in lux (I2C communication).
  - **MQ135**: Analog air quality monitoring (e.g., CO, NH3).
  - **HMC5883L**: Magnetic field strength and direction (I2C communication).

- **Real-Time Data Visualization:**
  - Dashboard displays all sensor readings.

- **Portable Design:**
  - Lightweight system with battery-powered operation.

---

## System Specifications
### Hardware Components
- **Microcontroller:** STM32 Nucleo L476RG with I2C and analog input support.
- **Display:** LCD (I2C communication).
- **Sensors:**
  - BME280
  - BH1750
  - MQ135
  - HMC5883L
- **Power Source:** USB or battery-powered.

### Operating Constraints
- **Sensor Calibration:** Periodic recalibration is required for accurate readings.
- **Air Quality Accuracy:** MQ135 readings are approximate and sensitive to environmental variations.
- **Display Latency:** Minimal latency in LCD updates may be observed.

### Known Limitations
- Limited memory on STM32 restricts additional features like data logging.
- MQ135’s accuracy is affected by temperature and humidity fluctuations.

---

## Setup and Usage
### 1. Hardware Connections
- **BME280, BH1750, HMC5883L:** Connect to I2C pins (SDA and SCL).
- **MQ135:** Connect to an analog input pin.
- **LCD:** Connect to the same I2C bus.

### 2. Software Requirements
- STM32CubeIDE for firmware development.
- HAL libraries for I2C, GPIO, and ADC interfaces.

### 3. Steps to Run the System
1. Connect all hardware components to the STM32 Nucleo board.
2. Load the firmware onto the microcontroller using STM32CubeIDE.
3. Power on the system using USB or battery.

---

## Test Plan and Results
### Test Cases
1. **BME280 Initialization and Data Acquisition:** Ensure temperature, humidity, and pressure readings are accurate.
2. **BH1750 Light Intensity Measurement:** Validate lux readings in controlled environments.
3. **MQ135 Air Quality Monitoring:** Test sensor response to varying air quality conditions.
4. **HMC5883L Magnetic Field Detection:** Verify compass functionality and magnetic field readings.

### Test Results
- All sensors initialized successfully and provided accurate readings.
- LCD updates were smooth and intuitive, with no observable lag.
- Air quality readings showed noticeable differences in varied conditions.

---

## Future Enhancements
- **Wireless Data Transmission:** Add Wi-Fi or Bluetooth for remote monitoring.
- **Data Logging:** Implement an SD card interface for storing sensor data.
- **Cloud Integration:** Enable IoT functionality for real-time analytics.
- **Enhanced UI:** Improve the LCD interface with graphs and animations.

---

## Parts List
| Component           | Approximate Cost |
| ------------------- | ---------------- |
| STM32 Nucleo L476RG | $5 - $10         |
| BME280              | $5 - $8          |
| BH1750              | $2 - $5          |
| MQ135               | $5 - $10         |
| HMC5883L            | $3 - $7          |
| TFT LCD             | $5 - $15         |
| **Total**           | **$33 - $65**    |

---

## Contributors
- **Victor Stafussi Granado**: Embedded Systems Design and Firmware Development


---

## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgments
- [Datasheets and Reference Manuals](#): BME280, BH1750, MQ135, HMC5883L, STM32.
- Open-source libraries for embedded systems development.
- STM32CubeIDE and HAL documentation for microcontroller interfacing.

---

For any questions or feedback, feel free to contact.

