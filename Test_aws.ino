// Test_aws.ino

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "config.h"
#include "aws_certs.h"

// WiFi and MQTT Clients
WiFiClientSecure wifiClientSecure;
PubSubClient mqttClient(wifiClientSecure);

// --- State Management ---
// Enum to define the system's current state
enum SystemState {
    WAITING_FOR_TRIGGER,
    PROCESSING_IMAGE,
    WAITING_FOR_RESPONSE
};
volatile SystemState currentState = WAITING_FOR_TRIGGER;

// Flag to be set by the interrupt when the IR sensor is triggered
volatile bool irSensorTriggered = false;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; // Mutex for safe access to the flag

// To handle timeouts if we don't get a response from AWS
unsigned long responseWaitStartTime = 0;
const unsigned long RESPONSE_TIMEOUT_MS = 20000; // 20-second timeout

// --- Function Declarations ---
void setupWiFi();
void setupCamera();
void setupMQTT();
void connectMQTT();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void takeAndSendPicture();
String base64_encode(const unsigned char *data, size_t input_length);
void handleAuthorized();
void handleUnauthorized();
void blinkLED(int pin, int times, int delay_ms);

/**
 * @brief Interrupt Service Routine (ISR) called on the RISING edge of the IR sensor pin.
 * This function is kept lean for speed. It just sets a flag.
 */
void IRAM_ATTR onIrSensorDetect() {
    // Only set the trigger flag if the system is ready for a new person
    if (currentState == WAITING_FOR_TRIGGER) {
        portENTER_CRITICAL_ISR(&mux);
        irSensorTriggered = true;
        portEXIT_CRITICAL_ISR(&mux);
    }
}

void setup() {
    Serial.begin(115200);
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector

    Serial.println("Initializing Face Recognition Door Lock System...");

    // Configure Pin Modes based on config.h
    pinMode(IR_SENSOR_PIN, INPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_WHITE_PIN, OUTPUT);
    pinMode(SOLENOID_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);

    // Set initial actuator states to OFF
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_WHITE_PIN, LOW);
    digitalWrite(SOLENOID_PIN, LOW); // Ensure lock is OFF
    digitalWrite(BUZZER_PIN, LOW);

    setupCamera();
    setupWiFi();
    setupMQTT();

    // Attach the interrupt to the IR sensor pin
    attachInterrupt(digitalPinToInterrupt(IR_SENSOR_PIN), onIrSensorDetect, RISING);

    Serial.println("\n✅ System Ready. Waiting for IR sensor trigger...");
}

void loop() {
    // Always maintain WiFi and MQTT connections for reliability
    if (!WiFi.isConnected()) {
        Serial.println("WiFi disconnected. Reconnecting...");
        setupWiFi(); // This will block until connected
    }
    if (!mqttClient.connected()) {
        connectMQTT();
    }
    // This is crucial for receiving messages from AWS
    mqttClient.loop();

    // The main trigger for the face recognition flow
    // Checks if the IR sensor has detected a person AND if the system is ready for a new face
    if (digitalRead(IR_SENSOR_PIN) == LOW && currentState == WAITING_FOR_TRIGGER) {
        
        // Change state to indicate we are busy processing a face
        currentState = WAITING_FOR_RESPONSE;
        
        Serial.println("IR Sensor Triggered! Capturing image for face recognition...");
        
        // Turn ON the white light to show the system is active, as requested
        digitalWrite(LED_WHITE_PIN, HIGH);
        
        // Call the function that takes a picture and sends it to your MAIN Lambda
        takeAndSendPicture(); 
    }

    // A small delay to keep the microcontroller from being overloaded
    delay(100);
}

/**
 * @brief Handles the logic for an authorized person.
 */
void handleAuthorized() {
    Serial.println("*****************************");
    Serial.println("* PERSON AUTHORIZED!     *");
    Serial.println("*****************************");

    // Blink Green LED 5 times
    blinkLED(LED_GREEN_PIN, 5, 200);

    // Activate Relay (Solenoid) for 1 second to unlock the door
    Serial.println("Unlocking door: Activating solenoid...");
    digitalWrite(SOLENOID_PIN, HIGH);
    delay(2000); // Keep solenoid on for 1 second
    digitalWrite(SOLENOID_PIN, LOW);
    delay(2000);
}

/**
 * @brief Handles the logic for an unauthorized person.
 */
void handleUnauthorized() {
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("!   PERSON NOT AUTHORIZED!  !");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

    // Blink Red LED 5 times and sound the buzzer simultaneously
    Serial.println("Access Denied: Activating red LED and buzzer...");
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_RED_PIN, HIGH);
        digitalWrite(BUZZER_PIN, HIGH);
        delay(200); // The "ON" time for the blink
        
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(BUZZER_PIN, LOW);
        
        // --- THIS IS THE FIX ---
        // Only apply the "OFF" delay if it's not the last blink
        if (i < 4) {
            delay(200);
        }
    }
}


/**
 * @brief Final callback function for MQTT messages. 
 * Handles all commands from the cloud and executes the correct hardware sequence.
 */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // Print the incoming message for debugging
    Serial.print("--- MQTT Message Arrived [");
    Serial.print(topic);
    Serial.println("] ---");
    char msg[length + 1];
    for (unsigned int i = 0; i < length; i++) { msg[i] = (char)payload[i]; }
    msg[length] = '\0';
    Serial.println(msg);

    // Parse the JSON message
    StaticJsonDocument<256> doc;
    deserializeJson(doc, msg);
    const char* action = doc["action"];

    if (!action) {
        Serial.println("No 'action' key in message.");
        return;
    }

    // --- SCENARIO 1: FACE RECOGNITION RESPONSE ---
    
    // Case 1A: Face was AUTHORIZED
    if (strcmp(action, "unlock") == 0 && currentState == WAITING_FOR_RESPONSE) {
        Serial.println("Result: Face recognition AUTHORIZED.");
        digitalWrite(LED_WHITE_PIN, LOW); // Turn OFF white light
        digitalWrite(LED_GREEN_PIN, HIGH); // Turn ON green light

        digitalWrite(SOLENOID_PIN, HIGH); // Unlock solenoid
        delay(7000); // Wait 7 seconds as requested
        digitalWrite(SOLENOID_PIN, LOW);  // Re-lock
        
        digitalWrite(LED_GREEN_PIN, LOW); // Turn OFF green light
        Serial.println("Solenoid triggered by face recognition.");
        
        // Reset the state machine to be ready for the next person
        currentState = WAITING_FOR_TRIGGER;
    }
    // Case 1B: Face was DENIED
    else if (strcmp(action, "deny") == 0 && currentState == WAITING_FOR_RESPONSE) {
        Serial.println("Result: Face recognition DENIED.");
        digitalWrite(LED_WHITE_PIN, LOW); // Turn OFF white light
        
        digitalWrite(LED_RED_PIN, HIGH); // Turn ON red light
        digitalWrite(BUZZER_PIN, HIGH);  // Turn ON buzzer
        delay(2000); // Keep them on for 2 seconds
        digitalWrite(LED_RED_PIN, LOW);  // Turn OFF red light
        digitalWrite(BUZZER_PIN, LOW);   // Turn OFF buzzer

        // Reset the state machine
        currentState = WAITING_FOR_TRIGGER;
    }

    // --- SCENARIO 2: REMOTE UNLOCK COMMANDS ---

    // Case 2A: App asks to start the remote unlock process (capture first)
    else if (strcmp(action, "capture_for_unlock") == 0) {
        Serial.println("Command: Capture photo for remote unlock log.");
        // This function takes a picture and sends it to the main Lambda with the "log_only" flag
        captureAndPublishLogImage();
    }
    // Case 2B: Cloud confirms the photo was logged and tells us to unlock
    else if (strcmp(action, "confirm_unlock") == 0) {
        Serial.println("Command: Confirmation received. Unlocking door remotely.");
        digitalWrite(LED_GREEN_PIN, HIGH); // Turn ON only the green light as requested
        
        digitalWrite(SOLENOID_PIN, HIGH); // Unlock solenoid
        delay(7000); // Keep lock open for 7s
        digitalWrite(SOLENOID_PIN, LOW);  // Re-lock
        
        digitalWrite(LED_GREEN_PIN, LOW); // Turn OFF green light
        Serial.println("Solenoid triggered by remote command.");
    }
}

// --- Utility and Setup Functions (Largely Unchanged) ---

void setupCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_HVGA;
    config.jpeg_quality = IMAGE_QUALITY;
    config.fb_count = 1;
    
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x\n", err);
        ESP.restart();
    }
    Serial.println("Camera initialized.");
}

void setupWiFi() {
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        if (millis() - startTime > 30000) {
            Serial.println("\nFailed to connect to WiFi. Restarting...");
            ESP.restart();
        }
    }
    Serial.println("\nWiFi connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void setupMQTT() {
    wifiClientSecure.setCACert(aws_root_ca_pem);
    wifiClientSecure.setCertificate(aws_device_certificate_pem);
    wifiClientSecure.setPrivateKey(aws_device_private_key_pem);
    mqttClient.setServer(AWS_IOT_ENDPOINT, 8883);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(MQTT_MAX_PACKET_SIZE);
    Serial.println("MQTT Client configured.");
}

void connectMQTT() {
    Serial.print("Connecting to AWS IoT MQTT Broker...");
    String clientId = "esp32-doorlock-client-";
    clientId += String(random(0xffff), HEX);

    while (!mqttClient.connected()) {
        if (mqttClient.connect(clientId.c_str())) {
            Serial.println("Connected!");
            if(mqttClient.subscribe(AWS_MQTT_TOPIC_SUBSCRIBE)){
                Serial.print("Subscribed to: "); Serial.println(AWS_MQTT_TOPIC_SUBSCRIBE);
            } else {
                Serial.print("Failed to subscribe to topic.");
            }
        } else {
            Serial.print("Failed, rc=");
            Serial.print(mqttClient.state());
            Serial.println(" Retrying in 5 seconds...");
            delay(5000);
        }
    }
}

void takeAndSendPicture() {
    if (!mqttClient.connected()) {
        Serial.println("MQTT not connected. Cannot send picture.");
        return;
    }

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        digitalWrite(LED_WHITE_PIN, LOW);
        currentState = WAITING_FOR_TRIGGER; // Reset on failure
        return;
    }
    
    Serial.printf("Picture taken! Size: %zu bytes\n", fb->len);

    if (fb->format != PIXFORMAT_JPEG) {
        Serial.println("Image format is not JPEG.");
        esp_camera_fb_return(fb);
        return;
    }

    String imageBase64 = base64_encode(fb->buf, fb->len);
    esp_camera_fb_return(fb); // IMPORTANT: free buffer immediately after use

    if (imageBase64.length() == 0) {
        Serial.println("Base64 encoding failed.");
        return;
    }
    
    DynamicJsonDocument jsonDoc(MQTT_MAX_PACKET_SIZE - 512);
    jsonDoc["image_data"] = imageBase64;
    
    String jsonBuffer;
    serializeJson(jsonDoc, jsonBuffer);
    
    if (jsonBuffer.length() > MQTT_MAX_PACKET_SIZE) {
        Serial.printf("ERROR: Payload size (%d) exceeds MQTT max packet size.\n", jsonBuffer.length());
        return;
    }
    
    Serial.print("Publishing image payload...");
    if (mqttClient.publish(AWS_MQTT_TOPIC_PUBLISH, jsonBuffer.c_str(), false)) {
        Serial.println("Published successfully.");
    } else {
        Serial.println("Publish FAILED!");
    }
}

void captureAndPublishLogImage() {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed for logging");
        return;
    }

    // Base64 encode the image
    String imageBase64 = base64_encode(fb->buf, fb->len);
    esp_camera_fb_return(fb); // Free buffer immediately

    if (imageBase64.length() == 0) {
        Serial.println("Base64 encoding failed for log image.");
        return;
    }

    // Create JSON payload with the log_only flag
    DynamicJsonDocument jsonDoc(MQTT_MAX_PACKET_SIZE - 512);
    jsonDoc["image_data"] = imageBase64;
    jsonDoc["log_only"] = true; // IMPORTANT: Add the flag

    String jsonBuffer;
    serializeJson(jsonDoc, jsonBuffer);

    // Publish to the standard image topic
    if (mqttClient.publish(AWS_MQTT_TOPIC_PUBLISH, jsonBuffer.c_str(), false)) {
        Serial.println("Successfully published log image to MQTT.");
    } else {
        Serial.println("Failed to publish log image.");
    }
}

void blinkLED(int pin, int times, int delay_ms) {
    for (int i = 0; i < times; i++) {
        digitalWrite(pin, HIGH);
        delay(delay_ms);
        digitalWrite(pin, LOW);
        if (i < times - 1) delay(delay_ms);
    }
}

const char b64_alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                            "abcdefghijklmnopqrstuvwxyz"
                            "0123456789+/";

String base64_encode(const unsigned char *data, size_t input_length) {
    String encoded_string;
    encoded_string.reserve((input_length * 4 / 3) + (input_length / 96) + 6);
    int i = 0, j = 0;
    unsigned char char_array_3[3], char_array_4[4];

    while (input_length--) {
        char_array_3[i++] = *(data++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;
            for (i = 0; (i < 4); i++) encoded_string += b64_alphabet[char_array_4[i]];
            i = 0;
        }
    }
    if (i) {
        for (j = i; j < 3; j++) char_array_3[j] = '\0';
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;
        for (j = 0; (j < i + 1); j++) encoded_string += b64_alphabet[char_array_4[j]];
        while ((i++ < 3)) encoded_string += '=';
    }
    return encoded_string;
}

