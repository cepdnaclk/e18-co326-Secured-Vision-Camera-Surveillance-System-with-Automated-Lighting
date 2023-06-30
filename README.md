___
# SECURED VISION CAMERA SURVEILLANCE SYSTEM WITH AUTOMATED LIGHTING
___

## INTRODUCTION
The Automated Security System provides an efficient and reliable solution for securing service rooms. The system recognizes any unauthorized access and takes the appropriate steps to both turn on the light and take pictures of the intruder. Additionally, the system offers a dashboard with access to the security system database and real-time monitoring. Overall, safeguarding service rooms may be done effectively and affordably with the Automated Security System.

## OBJECTIVES
- To detect any movement inside the service room and alert the system.
- To automatically activate the security features of the system, such as capturing images of the intruder and turning on the light bulb.
- To store the images of the intruder in the security system database for future reference.
- To provide a dashboard for monitoring the system and accessing the security system database.

[comment]: <> (## HARDWARE DESIGN)


## BASIC ARCHITECTURE
- When a person enters the service room, the motion sensor detects the movement and sends a signal to the ESP32 controller.
- The ESP32 controller receives the signal and sends it to the MQTT broker.
- The MQTT broker receives the signal and based on the value parameters set in the broker, passes the control action to the actuators.
- The light bulb is turned on automatically, and the camera captures the image of the person.
- The image is sent to the security system database for future reference.
- The dashboard provides real-time monitoring of the system and access to the security system database.
