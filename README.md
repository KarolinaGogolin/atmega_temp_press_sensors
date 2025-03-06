# Thermometer & Pressure Sensor with ATmega

## Overview
This project involves building an embedded system with the **ATmega microcontroller** to measure temperature and pressure using sensors. The system reads data from multiple sensors via **I2C communication**, processes the data, and displays the results on a serial monitor. This project helped me learn sensor integration, firmware development in **Embedded C**, and debugging techniques.

## Key Features
- Integrated temperature sensor (DS1621) and pressure sensor (BMP280) with ATmega.
- Displaying temperature data with **LED indicators** for integer or half-degree values.
- **Negative temperature indicator** with LED feedback.
- Serial output for monitoring real-time data.

## Components Used
- **Microcontroller**: ATmega328P
- **Sensors**:
  - DS1621 (Temperature Sensor)
  - BMP280 (Pressure Sensor)
- **External Components**:
  - MCP23017 (IO Expander)
  - 7-segment displays (optional, if you plan to add them)
  - LEDs (for negative temperature and half-degree display)
- **Programming Environment**: Arduino IDE (Embedded C)

## How It Works
- **I2C Communication**: The temperature and pressure data are read from the **DS1621** and **BMP280** sensors via I2C communication.
- **Temperature Calibration**: The raw data is converted into usable temperature values by compensating for sensor calibration.
- **Pressure Calibration**: Similarly, the pressure is read, compensated, and displayed.
- **LED Feedback**: LEDs are used to indicate certain states, such as whether the temperature is an integer or has a decimal part (0.5).
- **Serial Monitor**: The temperature and pressure values are output on the serial monitor every 10 seconds for real-time monitoring.

## Future Improvements
- Add a **7-segment displays** to show the temperature directly on the device.
- Implement **data logging** to store measurements over time.
- Optimize the code for lower power consumption for battery-operated systems.
- Add a **user interface** for adjusting settings or thresholds.
