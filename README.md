# Anemometer - Arduino Library for Wind Measurement

This project provides an Arduino library to measure wind speed and direction using four ultrasonic sensors. It supports both square and rectangular sensor arrangements and includes functions for calibration, speed measurement, direction calculation, and gust detection.

---

## Introduction

The Anemometer library allows you to read wind data with an Arduino or an ESP32. Using ultrasonic sensors, it measures both the wind speed (in m/s) and wind direction (in degrees, where 0° corresponds to North, 90° to East, etc.).  
The library supports:
- **Square Arrangement:** All four sensors are placed with the same distance.
- **Rectangular Arrangement:** Different distances in the North-South and West-East directions.

---

## Required Materials

- 4 Ultrasonic sensors (e.g., HC-SR04)

---

## Argument: Wiring and Hardware Connections  
**Explanation:**  
Each ultrasonic sensor has two essential pins:
- **Trigger Pin (trig):** Initiates the distance measurement.
- **Echo Pin (echo):** Receives the reflected ultrasonic pulse.
  
**Suggested Pin Assignments:**

| Direction | Trigger Pin Variable | Echo Pin Variable |
|-----------|----------------------|-------------------|
| North     | `ntrig`              | `necho`           |
| East      | `etrig`              | `eecho`           |
| South     | `strig`              | `secho`           |
| West      | `wtrig`              | `wecho`           |

Connect each sensor to the corresponding pins on the Arduino, ensuring correct wiring for power (5V/3.3V) and ground.

---

## Argument: Sensor Arrangement Constructors  
**Explanation:**  
There are two constructors to initialize the Anemometer object:
  **Square Arrangement Constructor:**  
   ```cpp
   Anemometer(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, unsigned int distance);
  ```
  **Rectangular Arrangement Constructor:**
  ```cpp
  Anemometer(unsigned int ntrig, unsigned int necho, unsigned int etrig, unsigned int eecho, unsigned int strig, unsigned int secho, unsigned int wtrig, unsigned int wecho, unsigned int distancenoso, unsigned int distanceweea);
  ```
---
## Methods:
The library provides several public

### Readstate
```cpp
void readstate()
```
Reads sensor data of the Ultrasonic Sensor

### Calibrate Sensor 
```cpp
void calibrate()
```
Calibrate the wind sensors only indoors or in calm conditions
### getspeed
```cpp
float getspeed()
```
Calculates and returns the wind speed in meters per second.
### getangle
```cpp
float getangle()
```
Calculates and returns the wind direction in degrees (0–360° where 90° = North and 0° = East).
### getguastswind
```cpp
float getgustswind()
```
Returns the maximum wind gust (highest measured wind speed) recorded during one readstate period.
