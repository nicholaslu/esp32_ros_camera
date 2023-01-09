#include <Arduino.h>
#include "esp_camera.h"
#include <WiFi.h>
// #include "dl_lib_matrix3d.h"

#define ROSSERIAL_ARDUINO_TCP

#include <ros.h>
#include <sensor_msgs/Image.h>
//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//

// Select camera model
// #define CAMERA_MODEL_WROVER_KIT // Has PSRAM
//#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
//#define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
//#define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
//#define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM

#include "camera_pins.h"

extern const char* ssid;
extern const char* password;
// Set the rosserial socket server IP address
IPAddress server(192,168,31,73);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher
sensor_msgs::Image img_msg;
ros::Publisher img_pub("esp32_cam", &img_msg);

char frame_id[] = "esp32_camera";
// uint32_t height = 640;
// uint32_t width = 480;
char encoding[] = "rgb8";
uint8_t is_bigendian = true;
// uint32_t step = 1920;
// uint8_t data[1920*480];
uint32_t array_size = 96*96*3;
// uint32_t array_size = 512;

// void startCameraServer();

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

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
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  // if(psramFound()){
  //   config.frame_size = FRAMESIZE_UXGA;
  //   config.jpeg_quality = 80;
  //   config.fb_count = 2;
  // } else {
  config.frame_size = FRAMESIZE_96X96;
  config.jpeg_quality = 64;
  config.fb_count = 1;
  // }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_96X96);

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // startCameraServer();

  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // Start to be polite
  nh.advertise(img_pub);

  Serial.print("Heap before malloc: ");
  Serial.println(ESP.getFreeHeap());

  img_msg.data = (uint8_t *) malloc(sizeof(uint8_t)*array_size);
  img_msg.data_length = array_size;

  Serial.print("Heap after malloc: ");
  Serial.println(ESP.getFreeHeap());
}

void loop() {
  // put your main code here, to run repeatedly:
  if (nh.connected()) {
    Serial.println("Connected");

    camera_fb_t * fb = NULL;
    fb = esp_camera_fb_get();
    if (!fb){
      Serial.println("Failed to capture");
      delay(1000);
      return;
    }

    img_msg.header.frame_id = frame_id;
    img_msg.header.stamp = nh.now();
    // img_msg.height = fb->height;
    img_msg.height = 96;
    // img_msg.width = fb->width;
    img_msg.width = 96;
    img_msg.encoding = encoding;
    img_msg.is_bigendian = is_bigendian;
    // img_msg.step = img_msg.width*3;
    img_msg.step = 96*3;

    // size_t size = img_msg.height*img_msg.width*3;
    // dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, img_msg.width, img_msg.height, 3);
    // uint8_t *out_buf = image_matrix->item;

    // uint8_t * data;
    // Serial.println(sizeof(uint8_t)*size);
    // img_msg.data = (uint8_t *) malloc(size);
    // img_msg.data_length = size;
    // data = (uint8_t *) malloc(size);
    bool cov = fmt2rgb888(fb->buf, array_size, fb->format, img_msg.data);
    // for (int i=0; i<array_size; i++){
    //   img_msg.data[i] = i%256;
    // }
    esp_camera_fb_return(fb);
    // bool cov = fmt2rgb888(fb->buf, size, fb->format, out_buf);
    if(!cov){
      Serial.println("Failed to convert");
      delay(1000);
      return;
    }
    Serial.println("Message ready to send");
    img_pub.publish(&img_msg);
    Serial.println("Message sent");
    // free(img_msg.data);
    
    // img_msg.data = data;

    // dl_matrix3du_free(image_matrix);
    // free(data);
    // size_t _jpg_buf_len;
    // uint8_t * _jpg_buf;
    // bool jpeg_converted = fmt2rgb888(fb, 80, &_jpg_buf, &_jpg_buf_len);
    // if (jpeg_converted){
    //   // cimg_msg.data = _jpg_buf;
    //   // cimg_msg.data = uint8_t (*_jpg_buf)[_jpg_buf_len];
    //   // uint8_t jpeg_data[_jpg_buf_len];
    //   uint8_t (*ptr)[_jpg_buf_len];
    //   ptr = (uint8_t(*)[_jpg_buf_len])_jpg_buf;
    //   // cimg_msg.data = *_jpg_buf;
    //   img_msg.data = *ptr;
    //   int i;
      // for (i = 0; i < 100; i++){
      //   Serial.println(cimg_msg.data[i]);
      // }
    // }
    // img_pub.publish( &img_msg );
    // esp_camera_fb_return(fb);
    // free(_jpg_buf);
  } else {
    Serial.println("Not Connected");
  }
  nh.spinOnce();
  // Loop exproximativly at 1Hz
  delay(1000);
}
