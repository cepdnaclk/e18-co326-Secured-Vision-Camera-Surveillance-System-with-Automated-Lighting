#include <MQTTClient.h>
#include <WiFi.h>
#include "esp_camera.h"

//camera pins for AI-Thinker camera model
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

// MQTT topic
#define ESP32CAM_PUBLISH_TOPIC "esp32/cam_0"

const int bufferSize = 40000;//requires the size of the data being sent

// Replace with your own network credentials
const char* ssid = "xxx";
const char* password = "xxxx";

// MQTT broker details
const char* mqtt_server = "xx.xx.xx.xx";
const int mqtt_port = 1883;
const char* mqtt_username = "xxx";
const char* mqtt_password = "xxx";

WiFiClient net;
MQTTClient client = MQTTClient(bufferSize);

void cameraInit(){
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;//640x480
  config.jpeg_quality = 10;
  config.fb_count = 2;//frame buffers

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
    return;
  }
}

void grabImage(){
  camera_fb_t * fb = esp_camera_fb_get();
  Serial.println("\n\n=====================");
  Serial.println("Publishing");
  Serial.println("=====================\n\n");
  
  if(fb != NULL && fb->format == PIXFORMAT_JPEG && fb->len < bufferSize){
    Serial.print("Image Length: ");
    Serial.print(fb->len);
    Serial.print("\t Publish Image: ");
    bool result = client.publish(ESP32CAM_PUBLISH_TOPIC, (const char*)fb->buf, fb->len);
    Serial.println(result);//true if published normally false if not
    
    if(!result){
      ESP.restart();
    }else{
      Serial.println("\n\n=====================");
      Serial.println("Successfully Published");
      Serial.println("=====================\n\n");
    }
  }
  esp_camera_fb_return(fb);
  delay(500);
}

void setupWifi(){
  WiFi.begin(ssid, password);

  Serial.println("\n\n=====================");
  Serial.println("Connecting to Wi-Fi");
  Serial.println("=====================\n\n");
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

void connectBroker(){
  client.begin(mqtt_server,mqtt_port,net);
  client.setCleanSession(true);

  Serial.println("\n\n=====================");
  String client_id = "esp32-client-";
  client_id += String(WiFi.macAddress());
  Serial.printf("The client %s connects to the private mqtt broker\n", client_id.c_str());
  
  while (!client.connect("ESP32CAM", mqtt_username, mqtt_password)) {
    Serial.println("Connecting to MQTT broker...");
    delay(5000);
  }
  Serial.println("=====================\n\n");

  Serial.println("\n\n=====================");
  Serial.println("MQTT Broker Connected!");
  Serial.println("=====================\n\n");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  cameraInit();
  setupWifi();
  connectBroker();
}

void loop() {
  // put your main code here, to run repeatedly:
  client.loop();
  if(client.connected()) grabImage();
}