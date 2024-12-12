#include <LiquidCrystal_I2C.h> //Enables interaction with an LCD screen using the I2C protocol.
#include <Arduino_FreeRTOS.h> //Allows the use of FreeRTOS, a real-time operating system, for task scheduling.
#include <semphr.h> // Provides support for semaphores for resource synchronization in FreeRTOS.
#include <stdint.h>
#include <DHT22.h> 

// ========================== Constants and Definitions ==========================

// LCD Configuration :Configures the LCD display to communicate using the I2C address 0x27 with a screen size of 16x2.
#define I2C_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// DHT22 Sensor Configuration
#define DHT_PIN 2                      // Digital pin connected to the DHT22
#define DHT_TYPE DHT22                 // Type of DHT sensor

// Pin Configuration
// photoresistor changes resistance based on light intensity
static const uint8_t photoresistorPin = A0;
static const uint8_t ALARM_PIN = 8;

// Task Priorities : Defines equal priorities for all tasks in the FreeRTOS environment. Higher numbers indicate higher priority.
#define TASK_PRIORITY_READ_LIGHT 1
#define TASK_PRIORITY_DISPLAY 1
#define TASK_PRIORITY_MONITOR 1
#define TASK_PRIORITY_ENV 1

// ========================== Global Variables and Objects ==========================

// Initialize LCD object
LiquidCrystal_I2C lcd(I2C_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// Initialize DHT22 object
DHT22 dht22(DHT_PIN);

typedef struct {
    float temperature;     // 4 bytes
    float humidity;        // 4 bytes
    uint16_t lightIntensity; // 2 bytes
} SensorData;

volatile SensorData sensorData = {0, 0, 0};
SemaphoreHandle_t sensorDataMutex;

// ========================== Stack Size Calculations ==========================

#define DISPLAY_TASK_STACK_SIZE 256
#define READ_TASK_STACK_SIZE 64
#define MONITOR_TASK_STACK_SIZE 64
#define ENV_TASK_STACK_SIZE 64

// ========================== Function Prototypes ==========================

// Task functions
void DisplayDataTask(void *pvParameters);
void ReadLightTask(void *pvParameters);
void MonitorThresholdsTask(void *pvParameters);
void ReadEnvironmentTask(void *pvParameters);
//tasks gain exclusive access to sensorData, ensuring consistent and accurate updates and reads.
// Helper functions
bool initializeSensors(void);
void triggerAlarm();

// ========================== Setup Function ==========================

void setup() {
    Serial.begin(9600);

    // Initialize LCD
    lcd.init();                        
    lcd.backlight();

    // Initialize pins
    pinMode(ALARM_PIN, OUTPUT);
    pinMode(photoresistorPin, INPUT);

    // Create  A FreeRTOS mutex to ensure thread-safe access to the sensorData
    sensorDataMutex = xSemaphoreCreateMutex();
    if (sensorDataMutex == NULL) {
        Serial.println(F("Mutex creation failed"));
    }
    // This mutex will act as a lock for accessing sensorData
    //Before a task modifies sensorData, it "takes" the mutex. This ensures no other task can access sensorData at the same time.
    //The portMAX_DELAY parameter means the task will wait indefinitely for the mutex to become available.
    // Create Tasks
    BaseType_t xReturned;
    
    xReturned = xTaskCreate(
        DisplayDataTask,
        "Display",
        DISPLAY_TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY_DISPLAY,
        NULL
    );
    
    if (xReturned != pdPASS) {
        Serial.println(F("Display Task creation failed"));
    }
           
    xReturned = xTaskCreate(
        ReadLightTask,
        "Light",
        READ_TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY_READ_LIGHT,
        NULL
    );
    
    if (xReturned != pdPASS) {
        Serial.println(F("Light Task creation failed"));
    }

    xReturned = xTaskCreate(
        MonitorThresholdsTask,
        "Monitor",
        ENV_TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY_ENV,
        NULL
    );
    
    if (xReturned != pdPASS) {
        Serial.println(F("Monitor Task creation failed"));
    }

    xReturned = xTaskCreate(
        ReadEnvironmentTask,
        "Environment",
        MONITOR_TASK_STACK_SIZE,
        NULL,
        TASK_PRIORITY_MONITOR,
        NULL
    );
        
    if (xReturned != pdPASS) {
        Serial.println(F("Environment Task creation failed"));
    }
}

void loop() {
    // Empty - Tasks handle everything
}

// ========================== Task Implementations ==========================

void DisplayDataTask(void *pvParameters) {
    uint16_t counter = 0;
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        // Direct reads are safe with volatile
        float temp = sensorData.temperature;
        float humidity = sensorData.humidity;
        uint16_t light = sensorData.lightIntensity;
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("T:"));
        lcd.print(temp, 1);
        lcd.print(F("C H:"));
        lcd.print(humidity, 1);
        lcd.print(F("%"));
        
        lcd.setCursor(0, 1);
        lcd.print(F("L:"));
        lcd.print(light);
        lcd.print(F(" C:"));
        lcd.print(counter++);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));
    }
}

void ReadLightTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    
    for (;;) {
        uint16_t sensorValue = analogRead(photoresistorPin);
        
        xSemaphoreTake(sensorDataMutex, portMAX_DELAY);
        sensorData.lightIntensity = sensorValue;
        xSemaphoreGive(sensorDataMutex);
        
        Serial.print(F("Light: "));
        Serial.println(sensorValue);
        
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

void MonitorThresholdsTask(void *pvParameters) {
    const uint8_t LIGHT_THRESHOLD = 30;
    const uint8_t TEMP_THRESHOLD = 60;
    const uint8_t HUMD_THRESHOLD = 30;
  
    for (;;) {
        // Direct reads are safe with volatile
        uint16_t light = sensorData.lightIntensity;
        float temp = sensorData.humidity;
        float humd = sensorData.temperature;
    
        bool alarmCondition = false;

        if (light > LIGHT_THRESHOLD) {
            alarmCondition = true;
        }

        if (temp > TEMP_THRESHOLD) {
            alarmCondition = true;
        }

        if (humd > HUMD_THRESHOLD) {
            alarmCondition = true;
        }

        if (alarmCondition) {
            triggerAlarm();
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void ReadEnvironmentTask(void *pvParameters) {
    for(;;) {
        float temp = dht22.getTemperature();
        float humidity = dht22.getHumidity();
        
        xSemaphoreTake(sensorDataMutex, portMAX_DELAY);
        sensorData.temperature = temp;
        sensorData.humidity = humidity;
        xSemaphoreGive(sensorDataMutex);
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

bool initializeSensors(void) {
    return true;
}

void triggerAlarm(void) {
    digitalWrite(ALARM_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500));
    digitalWrite(ALARM_PIN, LOW);
}
