// config.h

#ifndef CONFIG_H
#define CONFIG_H

// WiFi Credentials
#define WIFI_SSID "wel"
#define WIFI_PASSWORD "abcd1234*"

// AWS IoT Core Configuration
#define AWS_IOT_ENDPOINT "a2zoagruildosr-ats.iot.us-east-1.amazonaws.com"
#define AWS_MQTT_TOPIC_PUBLISH "esp32/door/image"
#define AWS_MQTT_TOPIC_SUBSCRIBE "esp32/door/result"

// --- Pinout Configuration (Updated as per your request) ---
// Sensors
#define IR_SENSOR_PIN     13 // Input: Detects presence (HIGH = object nearby)

// Actuators & Indicators
#define LED_GREEN_PIN     2  // Output: Blinks if face is recognized
#define LED_RED_PIN       15 // Output: Blinks if face is not recognized
#define LED_WHITE_PIN     12 // Output: ON during capture and recognition process
#define SOLENOID_PIN      14 // Output: Relay to trigger the solenoid lock
#define BUZZER_PIN        4  // Output: Sounds if face is not recognized

// Camera Pins (Standard for ESP32-CAM AI-Thinker)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Image Settings
#define IMAGE_QUALITY 15 // 0-63 lower means higher quality
#define IMAGE_WIDTH 320
#define IMAGE_HEIGHT 240

// MQTT Settings
#define MQTT_MAX_PACKET_SIZE 20000 // Increased for Base64 encoded images

#endif // CONFIG_H
