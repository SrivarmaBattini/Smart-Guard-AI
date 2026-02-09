# Smart-Guard-AI

## IoT-Based Smart Door Lock Using Face Recognition and AWS

The Smart-Guard-AI system operates as a smart door access control system which uses face recognition technology to enable secure automatic entry. The system uses an ESP32-CAM module which works together with AWS cloud services to perform identity verification before allowing access. The system uses biometric authentication methods which replace traditional access methods like keys and passwords to provide better protection and operational reliability.

The system operates based on an event-driven system design. The door area uses an IR sensor to detect people which activates the ESP32-CAM to capture a facial image. The system transfers the captured image to the AWS cloud in a secure manner. The cloud-based face recognition services compare the image with facial data which has been previously stored. The system establishes a comparison result which the device receives as an authentication decision.

The ESP32-CAM establishes door access through solenoid lock activation which confirms access through green LED signaling. The door remains locked when a person lacks authorization while the system uses a red LED and buzzer to show access denial. The security system uses IoT devices together with cloud intelligence to create an automated security solution, which provides real-time monitoring capabilities.


## Hardware Components Used

- **ESP32-CAM** – Main controller and camera module  
- **IR Sensor** – Presence detection  
- **Solenoid Lock** – Door locking mechanism  
- **Relay Module** – Controls solenoid lock  
- **Green LED** – Indicates authorized access  
- **Red LED** – Indicates unauthorized access  
- **Buzzer** – Audible alert for access denial  
- **Power Supply** – Provides required power  



## Cloud Services Used

- **AWS IoT Core** – Secure communication between device and cloud  
- **AWS Lambda** – Image processing and decision logic  
- **AWS Rekognition** – Face recognition and comparison  
- **AWS S3** – Image and log storage  

---
