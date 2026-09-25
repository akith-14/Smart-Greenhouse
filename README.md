# COMP50069 Smart Greenhouse

## Scenario 2 – Automated Commercial Micro-Climate Nursery

This repository contains the firmware and supporting files for an
ESP32-based Smart Greenhouse prototype developed for COMP50069 –
Hardware, Microcontrollers and Sensors.

## System Overview

The greenhouse has three operating modes:

- AUTO
- MANUAL OVERRIDE
- SAFETY

Priority:

SAFETY > MANUAL > AUTO

## Automatic Control

### Light Control

The LDR controls the supplemental grow LEDs.

- LDR < 400 → LEDs ON
- LDR > 700 → LEDs OFF
- 400–700 → previous LED state retained

### Temperature Control

The DHT22 temperature controls the ventilation servo.

- Temperature <= 28°C → vent closed
- 28°C–35°C → proportional opening
- Temperature >= 35°C → vent fully open

Humidity is monitored and displayed but does not control an actuator.

## Manual Override

The GPIO19 push button or UART command M enters Manual Override.

During Manual Override:

- Automatic routines are bypassed
- Grow LEDs are OFF
- Vent is locked fully open
- OLED displays MANUAL OVERRIDE

## Safety Mode

Invalid or disconnected DHT22 readings activate Safety Mode.

Safety response:

- Grow LEDs OFF
- Vent moves to logical 45°
- OLED displays DHT22 FAILED / CHECK SENSOR
- Manual and AUTO requests are rejected until recovery

After a valid DHT22 reading returns, the system automatically
returns to AUTO mode.

## Hardware

- ESP32 Dev Module
- DHT22 temperature/humidity sensor
- LDR
- 2 × LEDs
- SG90 servo
- SSD1306 I2C OLED
- Push button
- 10kΩ resistor
- LED current-limiting resistors

## Pin Configuration

| Component | ESP32 Pin |
|---|---|
| DHT22 DATA | GPIO33 |
| LDR ADC | GPIO34 |
| LED 1 | GPIO16 |
| LED 2 | GPIO17 |
| Servo signal | GPIO18 |
| Manual button | GPIO19 |
| OLED SDA | GPIO21 |
| OLED SCL | GPIO22 |

## UART Commands

- A – AUTO mode
- M – Manual Override
- + – Increase vent-start temperature
- - – Decrease vent-start temperature
- S – Print system status
- ? – Print help

UART baud rate: 115200

## Wokwi Simulation
Link
https://wokwi.com/projects/475341524571115521

## Repository Contents

- Smart_Greenhouse_Scenario2.ino – final ESP32 firmware
- diagram.json – Wokwi circuit definition
- libraries.txt – Wokwi library dependencies
- images/ – prototype and operating-mode evidence
- docs/ – project documentation
