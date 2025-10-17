#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include "PCF8574.h" 
#include <lvgl.h>
#include <Adafruit_CST8XX.h>

#include "esp_heap_caps.h"
#include "esp_system.h"

#define I2C_SDA_PIN 38
#define I2C_SCL_PIN 39
PCF8574 pcf8574(0x21);

#define SCREEN_BACKLIGHT_PIN 6
const int pwmFreq = 5000;
const int pwmChannel = 0;
const int pwmResolution = 8;

#define I2C_TOUCH_ADDR 0x15 

#define ENCODER_A_PIN 42    
#define ENCODER_B_PIN 4     
volatile uint8_t currentA = 0; 
volatile uint8_t lastA = 0;   
volatile uint8_t currentSW = 0; 
volatile uint8_t swPin = 0;  

Adafruit_CST8XX tsPanel = Adafruit_CST8XX();

static const uint16_t HOR_RES = 480;
static const uint16_t VER_RES = 480;

static lv_display_t *lv_disp = nullptr;
static lv_indev_t   *lv_indev = nullptr;

static uint8_t *buf1 = nullptr;
static uint8_t *buf2 = nullptr;

// Taken from sample code
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
  16 /* CS */, 2 /* SCK */, 1 /* SDA */,
  40 /* DE */, 7 /* VSYNC */, 15 /* HSYNC */, 41 /* PCLK */,
  46 /* R0 */, 3 /* R1 */, 8 /* R2 */, 18 /* R3 */, 17 /* R4 */,
  14 /* G0/P22 */, 13 /* G1/P23 */, 12 /* G2/P24 */, 11 /* G3/P25 */, 10 /* G4/P26 */, 9 /* G5 */,
  5 /* B0 */, 45 /* B1 */, 48 /* B2 */, 47 /* B3 */, 21 /* B4 */
);

Arduino_ST7701_RGBPanel *gfx = new Arduino_ST7701_RGBPanel(
  bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */,
  false /* IPS */, 480 /* width */, 480 /* height */,
  st7701_type5_init_operations, sizeof(st7701_type5_init_operations),
  true /* BGR */,
  10 /* hsync_front_porch(10) */, 4 /* hsync_pulse_width(8) */, 20 /* hsync_back_porch(50) */,
  10 /* vsync_front_porch(10) */, 4 /* vsync_pulse_width(8) */, 20 /* vsync_back_porch(20) */);



static void lv_make_sanity() {
  lv_obj_t *label = lv_label_create(lv_screen_active());
  lv_label_set_text(label, "Hello LVGL 9.3");
  lv_obj_center(label);

  lv_obj_t *spin = lv_spinner_create(lv_screen_active());
  lv_obj_set_size(spin, 36, 36);
  lv_obj_align(spin, LV_ALIGN_BOTTOM_MID, 0, -8);
}


void flush_callback(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    const int32_t w = area->x2 - area->x1 + 1;
    const int32_t h = area->y2 - area->y1 + 1;

    // treating the px_map buffer as uint16_t array for 16-bit RGB565 output
    #if (LV_COLOR_16_SWAP != 0)
        gfx->draw16bitBeRGBBitmap(area->x1, area->y1, reinterpret_cast<uint16_t *>(px_map), w, h);
    #else
        gfx->draw16bitRGBBitmap(area->x1, area->y1, reinterpret_cast<uint16_t *>(px_map), w, h);
    #endif

    lv_disp_flush_ready(disp);
}

static void touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
  if (tsPanel.touched()) {
        CST_TS_Point point = tsPanel.getPoint();
        if (point.z > 0) {  // point is being touched
            data->state = LV_INDEV_STATE_PRESSED;
            data->point.x = point.x;
            data->point.y = point.y;
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
        }
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }

  // Serial.println(touched ? "[Touch] touched" : "[Touch] not touched"); Was using for debugging in 1.28 inch, touched bool not used here
}

void initBacklight() {
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(SCREEN_BACKLIGHT_PIN, pwmChannel);
  ledcWrite(pwmChannel, 204);
}
  
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  pcf8574.pinMode(P0, OUTPUT); //tp RST
  pcf8574.pinMode(P2, OUTPUT);  //tp INT
  pcf8574.pinMode(P3, OUTPUT); //lcd power
  pcf8574.pinMode(P4, OUTPUT);  //lcd reset
  pcf8574.pinMode(P5, INPUT_PULLUP); //encoder SW

  if (!psramInit()) {
    Serial.println("PSRAM init failed");
  }
  Serial.printf("SPIRAM free: %u, largest block: %u\n",
                heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));

  Serial.print("Init pcf8574...\n");
  if (pcf8574.begin()) {
    Serial.println("pcf8574 OK");
  } else {
    Serial.println("pcf8574 KO");
  }


  pcf8574.digitalWrite(P3, HIGH);
  delay(100);

  // lcd reset
  pcf8574.digitalWrite(P4, HIGH);
  delay(100);
  pcf8574.digitalWrite(P4, LOW);
  delay(120);
  pcf8574.digitalWrite(P4, HIGH);
  delay(120);


  // touchpoint reset
  pcf8574.digitalWrite(P0, HIGH);
  delay(100);
  pcf8574.digitalWrite(P0, LOW);
  delay(120);
  pcf8574.digitalWrite(P0, HIGH);
  delay(120);

  
  pcf8574.digitalWrite(P2, HIGH);
  delay(120);

  gfx->begin();
  gfx->fillScreen(BLACK);
  gfx->setTextSize(2);
  gfx->setCursor(80, 100);

  if (!tsPanel.begin(&Wire, I2C_TOUCH_ADDR)) {
    Serial.println("No touchscreen found");
  } else {
    Serial.println("Touchscreen found");
  }

  pinMode(ENCODER_A_PIN, INPUT);
  pinMode(ENCODER_B_PIN, INPUT);
  lastA = digitalRead(ENCODER_A_PIN);

  lv_init();

  size_t buf_size = (size_t)HOR_RES * 40 /*lines*/ * 2 /*RGB565 bytes*/;
  buf1 = (uint8_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
  buf2 = (uint8_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
  if (!buf1 || !buf2) {
    Serial.println("[LVGL] Buffer alloc failed; falling back to single buffer");
    if (!buf1) buf1 = (uint8_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM);
  }

  lv_disp = lv_display_create(HOR_RES, VER_RES);
  lv_display_set_flush_cb(lv_disp, flush_callback);
  lv_display_set_buffers(lv_disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_indev = lv_indev_create();
  lv_indev_set_type(lv_indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(lv_indev, touchpad_read);

  lv_make_sanity();

  initBacklight();

  Serial.println("Setup done.");
}

void loop() {
  lv_timer_handler();
  delay(5);

  static uint32_t last = 0;
  if (millis() - last > 500) {
    uint16_t x, y;

    if (tsPanel.touched()) {
        CST_TS_Point point = tsPanel.getPoint();
        Serial.printf("[Touch] x=%u y=%u\n", point.x, point.y);
    }

    last = millis();
  }
}

