/* o objetivo deste codigo é detectar movimentos e registrar em fotos no cartao SD
 * source:
 * https://eloquentarduino.github.io/2020/01/motion-detection-with-esp32-cam-only-arduino-version/
 * 
 * consumo de bateria 
 * webcam server (exemplo da placa) = 0,07 A
 * webcam server fazendo streaming CIF = 0,21 A
 * rodando apenas o motion detection = 0,08 A

  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-take-photo-save-microsd-card
  
  IMPORTANT!!! 
   - Select Board "AI Thinker ESP32-CAM"
   - GPIO 0 must be connected to GND to upload a sketch
   - After connecting GPIO 0 to GND, press the ESP32-CAM on-board RESET button to put your board in flashing mode
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.


----  problema: Motion detected! >>> Capture Failed
*********/

#define CAMERA_MODEL_AI_THINKER

#include "esp_camera.h"
#include "camera_pins.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"
#include <EEPROM.h>            // read and write from flash memory

// define the number of bytes you want to access
#define EEPROM_SIZE 1

int pictureNumber = 0;

#define FRAME_SIZE FRAMESIZE_QVGA
#define WIDTH 320
#define HEIGHT 240
#define BLOCK_SIZE 10
#define W (WIDTH / BLOCK_SIZE)
#define H (HEIGHT / BLOCK_SIZE)
#define BLOCK_DIFF_THRESHOLD 0.2
#define IMAGE_DIFF_THRESHOLD 0.1
#define DEBUG 1


uint16_t prev_frame[H][W] = { 0 };
uint16_t current_frame[H][W] = { 0 };


bool setup_camera(framesize_t);
bool capture_still();
bool motion_detect();
void update_frame();
void print_frame(uint16_t frame[H][W]);

void setup() {

   WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
    Serial.begin(115200);
    Serial.println(setup_camera(FRAME_SIZE) ? "OK" : "ERR INIT");

  //Serial.println("Starting SD Card");
  if(!SD_MMC.begin()){
    Serial.println("SD Card Mount Failed");
    return;
  }
  
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  rtc_gpio_hold_en(GPIO_NUM_4);

}

void loop() {
    if (!capture_still()) {
        Serial.println("Failed capture");
        delay(3000);

        return;
    }

    if (motion_detect()) {
        Serial.println("Motion detected");
        fotografar_salvar();
    }

    update_frame();
    Serial.println("=================");
}

bool setup_camera(framesize_t frameSize) {   
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
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = frameSize;
    config.jpeg_quality = 12;
    config.fb_count = 2;   //feb2

    bool ok = esp_camera_init(&config) == ESP_OK;

    sensor_t *sensor = esp_camera_sensor_get();
    sensor->set_framesize(sensor, frameSize);

    return ok;
}

/******************************************** Capture image and do down-sampling */
bool capture_still() {
    camera_fb_t *frame_buffer = esp_camera_fb_get();

    if (!frame_buffer)
        return false;

    // set all 0s in current frame
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] = 0;


    // down-sample image in blocks
    for (uint32_t i = 0; i < WIDTH * HEIGHT; i++) {
        const uint16_t x = i % WIDTH;
        const uint16_t y = floor(i / WIDTH);
        const uint8_t block_x = floor(x / BLOCK_SIZE);
        const uint8_t block_y = floor(y / BLOCK_SIZE);
        const uint8_t pixel = frame_buffer->buf[i];
        const uint16_t current = current_frame[block_y][block_x];

        // average pixels in block (accumulate)
        current_frame[block_y][block_x] += pixel;
    }

    // average pixels in block (rescale)
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] /= BLOCK_SIZE * BLOCK_SIZE;

#if DEBUG
    Serial.println("Current frame:");
    print_frame(current_frame);
    Serial.println("---------------");
#endif

    return true;
}


/* Compute the number of different blocks If there are enough, then motion happened */
bool motion_detect() {
    uint16_t changes = 0;
    const uint16_t blocks = (WIDTH * HEIGHT) / (BLOCK_SIZE * BLOCK_SIZE);

    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            float current = current_frame[y][x];
            float prev = prev_frame[y][x];
            float delta = abs(current - prev) / prev;

            if (delta >= BLOCK_DIFF_THRESHOLD) {
#if DEBUG
                Serial.print("diff\t");
                Serial.print(y);
                Serial.print('\t');
                Serial.println(x);
#endif

                changes += 1;
            }
        }
    }

    Serial.print("Changed ");
    Serial.print(changes);
    Serial.print(" out of ");
    Serial.println(blocks);

    return (1.0 * changes / blocks) > IMAGE_DIFF_THRESHOLD;
}


/**
 * Copy current frame to previous
 */
void update_frame() {
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            prev_frame[y][x] = current_frame[y][x];
        }
    }
}

/**
 * For serial debugging
 * @param frame
 */
void print_frame(uint16_t frame[H][W]) {
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            Serial.print(frame[y][x]);
            Serial.print('\t');
        }

        Serial.println();
    }
}

void fotografar_salvar() {
//mudar para configuracoes de foto de maior qualidade
    sensor_t *sensor = esp_camera_sensor_get();  //FEB2
    sensor->set_pixformat(sensor, PIXFORMAT_JPEG);  //FEB2
    sensor->set_framesize(sensor, FRAMESIZE_UXGA);  //FEB2
    sensor->set_quality(sensor, 10);  //FEB2

//          config.pixel_format = PIXFORMAT_JPEG;
//          config.frame_size = FRAMESIZE_UXGA; // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
//          config.jpeg_quality = 10;
//          config.fb_count = 2;
//          bool ok = esp_camera_init(&config) == ESP_OK;

  camera_fb_t * fb = NULL;
  // Take Picture with Camera
  fb = esp_camera_fb_get();  
    Serial.print("JPEG buffer len: ");  //FEB2
    Serial.println(fb->len);  //FEB2
  if(!fb) {
    Serial.println("Camera capture failed");
    return;
  }
  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.read(0) + 1;

  // Path where new picture will be saved in SD Card
  String path = "/picture" + String(pictureNumber) +".jpg";

  fs::FS &fs = SD_MMC; 
  Serial.printf("Picture file name: %s\n", path.c_str());
  
  File file = fs.open(path.c_str(), FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file in writing mode");
  } 
  else {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved file to path: %s\n", path.c_str());
    EEPROM.write(0, pictureNumber);
    EEPROM.commit();
  }
  file.close();
  esp_camera_fb_return(fb); 

//voltar configuracoes de qualidade para apenas motion detect
    // switch back to GRAY
    sensor->set_pixformat(sensor, PIXFORMAT_GRAYSCALE);  //FEB2
    sensor->set_framesize(sensor, FRAME_SIZE);  //FEB2
    sensor->set_quality(sensor, 12);  //FEB2
    
  delay(1000);
}
