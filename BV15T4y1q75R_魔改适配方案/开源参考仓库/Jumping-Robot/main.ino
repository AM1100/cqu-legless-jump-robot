// Main.ino — XIAO ESP32S3 Sense

#include <WiFi.h>
#include "esp_camera.h"

// ===== Access Point Credentials =====
const char* ssid1     = "XIAO-S3 Robot";
const char* password1 = "1234567890";

// Provided by app_httpd.cpp
extern void startCameraServer();
extern void robot_setup();
extern void robot_stop();
extern volatile unsigned int motor_speed;
extern uint8_t robo;
extern volatile unsigned long previous_time;
extern volatile unsigned long move_interval;

// ===== XIAO ESP32S3 Sense camera pin map (inlined) =====
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39

#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13

static void init_camera()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;  // camera uses LEDC ch0
  config.ledc_timer   = LEDC_TIMER_0;    // camera uses LEDC timer0

  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;

  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Keep QVGA for smooth FPS, bump quality slightly better than before
  if (psramFound()) {
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 8;   // lower = better quality
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_QVGA;
    config.jpeg_quality = 10;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    while (true) delay(1000);
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, FRAMESIZE_QVGA);
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);

    // Optional mild tuning (safe defaults)
    // You can set these back to 0 if you don't like it
    s->set_contrast(s, 1);
    s->set_saturation(s, 1);

    // Keep sensor quality aligned with config (UI slider can change it later)
    s->set_quality(s, psramFound() ? 8 : 10);
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  init_camera();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid1, password1);

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  startCameraServer();
  robot_setup();

  previous_time = millis();
  Serial.println("Ready. Open the AP IP in your browser.");
}

void loop()
{
  // Auto-stop after move_interval unless joystick keeps refreshing previous_time
  if (robo) {
    unsigned long currentMillis = millis();
    if (currentMillis - previous_time >= move_interval) {
      previous_time = currentMillis;
      robot_stop();
      robo = 0;
    }
  }

  delay(1);
  yield();
}
