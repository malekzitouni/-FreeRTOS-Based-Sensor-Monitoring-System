# -FreeRTOS-Based-Sensor-Monitoring-System

### **README: FreeRTOS-Based Sensor Monitoring System**

---

## **Project Overview**

This project implements a **FreeRTOS-based sensor monitoring system** for environmental and light measurements. It utilizes an LCD for displaying real-time data, a DHT22 sensor for temperature and humidity monitoring, and a photoresistor for light intensity measurement. Alarms are triggered when thresholds are exceeded. Tasks are scheduled using FreeRTOS, ensuring efficient resource utilization.

---

## **Features**

- **Temperature & Humidity Monitoring**:
  - Reads data from a DHT22 sensor.
  - Displays real-time temperature and humidity values on the LCD.
  
- **Light Intensity Monitoring**:
  - Reads data from a photoresistor.
  - Displays light intensity on the LCD.

- **Threshold Monitoring**:
  - Compares readings against predefined thresholds.
  - Activates an alarm if values exceed the thresholds.

- **Multitasking with FreeRTOS**:
  - Separate tasks for data reading, threshold monitoring, and display updates.
  - Ensures smooth, non-blocking operation.

---

## **Hardware Requirements**

1. **Microcontroller**: Arduino Uno or compatible board.
2. **Sensors**:
   - DHT22 (Temperature and Humidity Sensor).
   - Photoresistor (for light intensity measurement).
3. **LCD**: 16x2 LCD with I2C interface.
4. **Alarm**: LED or buzzer connected to a digital pin.
5. **Other Components**:
   - Resistors for photoresistor and pull-ups.
   - Breadboard and jumper wires.

---

## **Software Requirements**

1. **Arduino IDE**: Version 1.8 or later.
2. **Libraries**:
   - `LiquidCrystal_I2C`: For LCD communication.
   - `DHT22`: For interfacing with the DHT22 sensor.
   - `Arduino_FreeRTOS`: For implementing FreeRTOS.
3. **FreeRTOS**:
   - Used for task scheduling and inter-task communication.

---

## **System Design**

### **Tasks and Responsibilities**
1. **Display Task**:
   - Updates the LCD with temperature, humidity, light intensity, and a counter.
   - Runs every 2 seconds.

2. **Read Light Task**:
   - Reads light intensity from the photoresistor.
   - Updates shared `sensorData` structure.
   - Runs every 100 milliseconds.

3. **Monitor Thresholds Task**:
   - Monitors light, temperature, and humidity against thresholds.
   - Activates alarm if thresholds are exceeded.
   - Runs every 500 milliseconds.

4. **Read Environment Task**:
   - Reads temperature and humidity from the DHT22 sensor.
   - Updates shared `sensorData` structure.
   - Runs every 2 seconds.

### **Synchronization**
- **Mutex**: Used to synchronize access to the shared `sensorData` structure, preventing race conditions.

---

## **Wiring Diagram**

| Component         | Microcontroller Pin |
|--------------------|---------------------|
| DHT22 Data Pin     | D2                 |
| Photoresistor      | A0                 |
| LCD SDA            | A4                 |
| LCD SCL            | A5                 |
| Alarm (LED/Buzzer) | D8                 |

---

## **Setup Instructions**

1. **Hardware Connections**:
   - Connect the DHT22 sensor, photoresistor, LCD, and alarm as per the wiring diagram.

2. **Library Installation**:
   - Install the required libraries (`LiquidCrystal_I2C`, `DHT22`, `Arduino_FreeRTOS`) via the Arduino Library Manager.

3. **Code Upload**:
   - Open the provided `.ino` file in the Arduino IDE.
   - Select the correct board and port.
   - Upload the code.

4. **Monitor Outputs**:
   - Observe real-time readings on the LCD.
   - Use the Serial Monitor (at 9600 baud) for debug messages.

---

## **Threshold Configuration**

- **Light Threshold**: 30
- **Temperature Threshold**: 60°C
- **Humidity Threshold**: 30%

Modify these thresholds in the `MonitorThresholdsTask()` function if needed.

---

## **Project File Structure**

```
/SensorMonitoringProject
    |-- SensorMonitoring.ino       # Main Arduino source file
    |-- README.md                  # Project documentation
    |-- Libraries/
        |-- LiquidCrystal_I2C/     # Library for LCD
        |-- Arduino_FreeRTOS/      # FreeRTOS library
        |-- DHT22/                 # Library for DHT22 sensor
```

---

## **Future Enhancements**

1. Add support for more sensors (e.g., CO2, soil moisture).
2. Implement wireless data transmission (e.g., via Bluetooth or Wi-Fi).
3. Enhance user interface with a keypad for dynamic threshold configuration.
4. Expand the system for multiple sensor nodes.

---

## **License**

This project is open-source under the MIT License. Feel free to modify and use it for personal or educational purposes.

---

Enjoy building your **FreeRTOS-Based Sensor Monitoring System**! 🚀
