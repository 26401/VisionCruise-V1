#include "camera_setting.h"
#include <WiFi.h>
#include <WebServer.h>
#include <esp_camera.h>

WebServer server(80);

// WiFi配置
const char* ssid = "ESP32-S3-CAM";
const char* password = "12345678";

void setup() {
  Serial.begin(115200);
  
  // 摄像头配置（使用头文件中的定义）
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
  config.xclk_freq_hz = XCLK_FREQ_HZ;
  
  // 修改这里：尝试不同的像素格式
  config.pixel_format = PIXFORMAT_RGB565;  // 改为 RGB565 或其他格式
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 1;
  }

  // 初始化摄像头
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("摄像头初始化失败: 0x%x", err);
    
    // 尝试其他像素格式
    Serial.println("尝试使用 YUV422 格式...");
    config.pixel_format = PIXFORMAT_YUV422;
    err = esp_camera_init(&config);
    
    if (err != ESP_OK) {
      Serial.printf("YUV422 也失败: 0x%x", err);
      
      // 最后尝试 GRAYSCALE
      Serial.println("尝试使用 GRAYSCALE 格式...");
      config.pixel_format = PIXFORMAT_GRAYSCALE;
      err = esp_camera_init(&config);
    }
  }

  if (err != ESP_OK) {
    Serial.printf("所有格式尝试都失败: 0x%x", err);
    return;
  }

  Serial.println("摄像头初始化成功");
  
  // 获取并显示摄像头信息
  sensor_t *s = esp_camera_sensor_get();
  Serial.print("摄像头传感器: ");
  Serial.println(s->id.PID);
  Serial.print("当前像素格式: ");
  switch(config.pixel_format) {
    case PIXFORMAT_RGB565: Serial.println("RGB565"); break;
    case PIXFORMAT_YUV422: Serial.println("YUV422"); break;
    case PIXFORMAT_GRAYSCALE: Serial.println("GRAYSCALE"); break;
    case PIXFORMAT_JPEG: Serial.println("JPEG"); break;
    default: Serial.println("未知格式"); break;
  }

  // 设置WiFi热点
  WiFi.softAP(ssid, password);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("热点IP: ");
  Serial.println(myIP);

  // 设置Web路由 - 简化版本，先确保基础功能正常
  server.on("/", []() {
    String html = "<html><head><title>ESP32-CAM</title></head>";
    html += "<body><h1>ESP32-S3-CAM 测试页面</h1>";
    html += "<p>摄像头状态: 运行中</p>";
    html += "<p><a href='/jpg'>查看照片</a></p>";
    html += "</body></html>";
    server.send(200, "text/html", html);
  });

  server.on("/jpg", handleJPG);

  server.begin();
  Serial.println("HTTP服务器已启动");
}

void loop() {
  server.handleClient();
  delay(10);
}

// 修改后的handleJPG函数
void handleJPG() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "获取帧失败");
    return;
  }
  
  // 根据像素格式发送不同的内容类型
  const char* content_type = "image/jpeg";
  
  // 如果不是JPEG格式，可能需要转换或使用其他MIME类型
  // 这里简化处理，实际可能需要格式转换
  
  String imageData = "";
  imageData.reserve(fb->len);
  for (size_t i = 0; i < fb->len; i++) {
    imageData += (char)fb->buf[i];
  }
  
  server.send(200, content_type, imageData);
  esp_camera_fb_return(fb);
}