# Health Monitoring System

## Overview
An IoT-based wearable health monitoring system built using an ESP32 microcontroller.  
The system collects vital signs and motion data, shows readings on an OLED display, and sends data to ThingSpeak for real-time monitoring and alerts.

## Features
- 🌡 Body temperature monitoring (DS18B20)
- ❤️ Heart rate & SpO2 monitoring (MAX3010x)
- 🏃 Motion/activity monitoring (MPU6050) + fall-risk indication
- 📟 Live readings on OLED display
- ☁️ Cloud logging and visualization on ThingSpeak
- ✉️ Automatic email alerts using ThingSpeak React when values exceed thresholds

## System Components
- ESP32 (processing + WiFi communication)
- DS18B20 Temperature Sensor (OneWire)
- MAX3010x Pulse Oximeter (I2C)
- OLED Display (I2C)

## Cloud Integration
Sensor readings are uploaded to ThingSpeak via HTTP:
- Field 1: Heart Rate (BPM)
- Field 2: SpO2 (%)
- Field 3: Temperature (°C)

ThingSpeak visualizations include time-series plots, scatter/correlation plots, and histograms for distribution analysis.

## Tools & Technologies
- ESP32 / Arduino IDE
- ThingSpeak (data storage, charts, React alerts)
- Wokwi simulation 
- Libraries: Wire, WiFi, HTTPClient, OneWire, DallasTemperature, Adafruit_GFX, Adafruit_SSD1306, MAX3010x libraries

## How It Works
1. ESP32 reads data from sensors (temperature, HR/SpO2, motion).
2. Values are shown locally on the OLED screen.
3. Data is sent to ThingSpeak periodically.
4. Alerts are triggered via ThingSpeak React when readings exceed thresholds.
