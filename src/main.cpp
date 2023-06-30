#include <MQTTClient.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

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

// MQTT topics
#define ESP32CAM_ENABLE "UoP_CO_326_E18_16_esp32/enable"
#define ESP32CAM_PUBLISH_IMAGE "UoP_CO_326_E18_16_esp32/cam"
#define ESP32CAM_NORMAL_IMAGE "UoP_CO_326_E18_16_esp32/norm_cam"
#define ESP32CAM_NEGATIVE_IMAGE "UoP_CO_326_E18_16_esp32/neg_cam"
#define ESP32CAM_GRAYSCALE_IMAGE "UoP_CO_326_E18_16_esp32/gray_cam"
#define ESP32CAM_REDTINT_IMAGE "UoP_CO_326_E18_16_esp32/red_cam"
#define ESP32CAM_BLUETINT_IMAGE "UoP_CO_326_E18_16_esp32/blue_cam"
#define ESP32CAM_GREENTINT_IMAGE "UoP_CO_326_E18_16_esp32/green_cam"
#define ESP32CAM_SEPIA_IMAGE "UoP_CO_326_E18_16_esp32/sepia_cam"
#define ESP32CAM_MIRROR_IMAGE "UoP_CO_326_E18_16_esp32/mirror_cam"
#define ESP32CAM_FLIP_IMAGE "UoP_CO_326_E18_16_esp32/flip_cam"

bool isCameraEnabled=false;

const int bufferSize = 40000;//requires the size of the data being sent

// configure LED pin
const int LED_PIN=13;

int spe_effect=0;
int mirror=0;
int flip=0;

// Replace with your own network credentials
const char* ssid = "xxx";
const char* password = "xxx";

// MQTT broker details
const char* mqtt_server = "xx.xx.xx.xx";
const int mqtt_port = 1883;
const char* mqtt_username = "xx";
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
  sensor_t * s = esp_camera_sensor_get();

  // Serial.println("special effect value: ");
  // Serial.println(spe_effect);

  s->set_brightness(s, 0);     // -2 to 2
  s->set_contrast(s, 0);       // -2 to 2
  s->set_saturation(s, 0);     // -2 to 2
  s->set_special_effect(s, spe_effect); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
  s->set_aec2(s, 0);           // 0 = disable , 1 = enable
  s->set_ae_level(s, 0);       // -2 to 2
  s->set_aec_value(s, 300);    // 0 to 1200
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
  s->set_agc_gain(s, 0);       // 0 to 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6
  s->set_bpc(s, 0);            // 0 = disable , 1 = enable
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable
  s->set_hmirror(s, mirror);        // 0 = disable , 1 = enable
  s->set_vflip(s, flip);          // 0 = disable , 1 = enable
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable


  Serial.println("\n\n=====================");
  Serial.println("Publishing");
  Serial.println("=====================\n\n");
  
  if(fb != NULL && fb->format == PIXFORMAT_JPEG && fb->len < bufferSize){
    Serial.print("Image Length: ");
    Serial.print(fb->len);
    Serial.print("\t Publish Image: ");
    bool result = client.publish(ESP32CAM_PUBLISH_IMAGE, (const char*)fb->buf, fb->len);
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

void callback(String &topic, String &payload) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  Serial.println(payload);
  
  if(String(topic)==ESP32CAM_ENABLE){
    Serial.print("Camera ");
    if(payload == "true"){
      Serial.println("on");
      digitalWrite(LED_PIN, HIGH);
      isCameraEnabled = true;
    }else if(payload == "false"){
      Serial.println("off");
      digitalWrite(LED_PIN, LOW);
      isCameraEnabled = false;  
    }
  }else if(String(topic)==ESP32CAM_NORMAL_IMAGE){
    Serial.print("Negative mode ");
    if(payload == "true"){
      Serial.println("on");
      // digitalWrite(33, LOW);
      spe_effect=0;
    }
    else if(payload == "false"){
      Serial.println("off");
      // digitalWrite(33, HIGH);
      spe_effect=0;
    }
  }else if(String(topic)==ESP32CAM_NEGATIVE_IMAGE){
    Serial.print("Negative mode ");
    if(payload == "true"){
      Serial.println("on");
      // digitalWrite(33, LOW);
      spe_effect=1;
    }
    else if(payload == "false"){
      Serial.println("off");
      // digitalWrite(33, HIGH);
      spe_effect=0;
    }
  }else if(String(topic)==ESP32CAM_GRAYSCALE_IMAGE){
    Serial.print("Grayscale mode ");
    if(payload == "true"){
      Serial.println("on");
      // digitalWrite(33, LOW);
      spe_effect=2;
    }
    else if(payload == "false"){
      Serial.println("off");
      // digitalWrite(33, HIGH);
      spe_effect=0;
    }
  }else if(String(topic)==ESP32CAM_REDTINT_IMAGE){
    Serial.print("Red Tint mode ");
    if(payload == "true"){
      Serial.println("on");
      // digitalWrite(33, LOW);
      spe_effect=3;
    }
    else if(payload == "false"){
      Serial.println("off");
      // digitalWrite(33, HIGH);
      spe_effect=0;
    }
  }else if(String(topic)==ESP32CAM_BLUETINT_IMAGE){
    Serial.print("Blue Tint mode ");
    if(payload == "true"){
      Serial.println("on");
      // digitalWrite(33, LOW);
      spe_effect=5;
    }else if(payload == "false"){
      Serial.println("off");
      // digitalWrite(33, HIGH);
      spe_effect=0;
    }
  }else if(String(topic)==ESP32CAM_GREENTINT_IMAGE){
    Serial.print("Green Tint mode ");
    if(payload == "true"){
      Serial.println("on");
      // digitalWrite(33, LOW);
      spe_effect=4;
    }
    else if(payload == "false"){
      Serial.println("off");
      // digitalWrite(33, HIGH);
      spe_effect=0;
    }
  }else if(String(topic)==ESP32CAM_SEPIA_IMAGE){
    Serial.print("Sepia mode ");
    if(payload == "true"){
      Serial.println("on");
      // digitalWrite(33, LOW);
      spe_effect=6;
    }
    else if(payload == "false"){
      Serial.println("off");
      // digitalWrite(33, HIGH);
      spe_effect=0;
    }
  }else if(String(topic)==ESP32CAM_MIRROR_IMAGE){
    Serial.print("Mirror mode ");
    if(payload == "true"){
      Serial.println("on");
      // digitalWrite(33, LOW);
      mirror=1;
    }
    else if(payload == "false"){
      Serial.println("off");
      // digitalWrite(33, HIGH);
      mirror=0;
    }
  }else if(String(topic)==ESP32CAM_FLIP_IMAGE){
    Serial.print("Flip mode ");
    if(payload == "true"){
      Serial.println("on");
      // digitalWrite(33, LOW);
      flip=1;
    }
    else if(payload == "false"){
      Serial.println("off");
      // digitalWrite(33, HIGH);
      flip=0;
    }
  }else{
    Serial.println("No such topic");
  }
}

void connectBroker(){
  client.begin(mqtt_server,mqtt_port,net);
  client.setCleanSession(true);
  client.onMessage(callback);

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
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin(115200);
  cameraInit();
  setupWifi();
  connectBroker();
  client.subscribe(ESP32CAM_ENABLE);
  client.subscribe(ESP32CAM_NORMAL_IMAGE);
  client.subscribe(ESP32CAM_NEGATIVE_IMAGE);
  client.subscribe(ESP32CAM_NORMAL_IMAGE);
  client.subscribe(ESP32CAM_GRAYSCALE_IMAGE);
  client.subscribe(ESP32CAM_REDTINT_IMAGE);
  client.subscribe(ESP32CAM_BLUETINT_IMAGE);
  client.subscribe(ESP32CAM_GREENTINT_IMAGE);
  client.subscribe(ESP32CAM_SEPIA_IMAGE);
  client.subscribe(ESP32CAM_MIRROR_IMAGE);
  client.subscribe(ESP32CAM_FLIP_IMAGE);
}


void loop() {
  // put your main code here, to run repeatedly:
  client.loop();
  // pinMode(PIR_PIN,INPUT_PULLUP);
  // int motion=digitalRead(PIR_PIN);
  // Serial.println(motion);
  // if(motion==1){
  //   Serial.println("\n\n=====================");
  //   Serial.println("Motion Detected!");
  //   Serial.println("=====================\n\n");
  // }
  if(isCameraEnabled && client.connected() ){
    // Serial.println("\n\n=====================");
    // Serial.println("Motion Detected!");
    // Serial.println("=====================\n\n");
    grabImage();
    delay(1000);
  } 

}
