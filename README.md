# M.D.P.T.s---PROJECT
A repository documenting my collection of hobbyist satellites under the M. D. P. T. s program (Multiple Deployment Platform Tiny Satellite).

## Breadboard Prototype Section:
A collection of basic CanSat breadboard prototypes and flight data analysis tools. Includes beginner firmware for ESP32-based atmospheric probes with a variety of sensors (BMI160, BMP280+AHT20, etc.), real-time 3D visualization , These prototypes were built just for rapid iteration, and R&D, not for active deployment.

## MDPTs Breadboard v1
- Minimal setup with BMP280 + AHT20, BMI160 modules
- Initial working prototype
- BMP280 + AHT20 + BMI160
- USB telemetry working
- SD card support (in progress)
- LED status indicator
## Files

### `cansat_simple.ino`
Arduino firmware. Reads sensors, streams telemetry over USB at 20 Hz.

### `cansat_dashboard_v2.html`
Web dashboard (Chrome). Connect USB to see live 3D rocket and graphs.

## Wiring
- GPIO8 (SDA) + GPIO9 (SCL) → all I2C sensors
- GPIO10 → LED (220Ω resistor to GND)
- 3V3 → VCC, GND → GND on all sensors

### Data Output
Comma-separated values sent over USB:
- timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw, temp, humidity, pressure, altitude, vertical_speed

## Wiring
- GPIO8 (SDA) → all I2C sensors
- GPIO9 (SCL) → all I2C sensors  
- GPIO10 → LED (through 220Ω resistor)
- 3V3 → VCC on all sensors
- GND → GND on all sensors

## Acknowledgments
- Dashboard created with Claude (Anthropic AI)

