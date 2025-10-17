#include <Arduino.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include <lvgl.h>
#include <Adafruit_NeoPixel.h> 
#include <Wire.h>
#include "CST816D.h"

#define TP_I2C_SDA_PIN 6
#define TP_I2C_SCL_PIN 7
#define I2C_SDA_PIN    38
#define I2C_SCL_PIN    39

#define TP_INT 5
#define TP_RST 13

#define POWER_LIGHT_PIN 40  

#define SCREEN_BACKLIGHT_PIN 46
const int pwmFreq = 5000;
const int pwmChannel = 0;
const int pwmResolution = 8;

#define LED_PIN 48
#define LED_NUM 5
Adafruit_NeoPixel led(LED_NUM, LED_PIN, NEO_GRB + NEO_KHZ800);

class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI _bus;
public:
  LGFX(void) {
    {
      auto cfg = _bus.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 80000000;
      cfg.freq_read   = 20000000;
      cfg.spi_3wire   = true;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = 10;
      cfg.pin_mosi    = 11;
      cfg.pin_miso    = -1;
      cfg.pin_dc      = 3;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    { 
      auto cfg = _panel.config();
      cfg.pin_cs           = 9;
      cfg.pin_rst          = 14;
      cfg.pin_busy         = -1;
      cfg.memory_width     = 240;
      cfg.memory_height    = 240;
      cfg.panel_width      = 240;
      cfg.panel_height     = 240;
      cfg.offset_x         = 0;
      cfg.offset_y         = 0;
      cfg.offset_rotation  = 0;
      cfg.dummy_read_pixel = 8;
      cfg.dummy_read_bits  = 1;
      cfg.readable         = false;
      cfg.invert           = true;
      cfg.rgb_order        = false;
      cfg.dlen_16bit       = false;
      cfg.bus_shared       = false;
      _panel.config(cfg);
    }
    setPanel(&_panel);
  }
};
static LGFX gfx;

static lv_display_t *lv_disp = nullptr;
static lv_indev_t   *lv_indev = nullptr;

static uint8_t *buf1 = nullptr;
static uint8_t *buf2 = nullptr;
static const uint16_t HOR_RES = 240;
static const uint16_t VER_RES = 240;

static CST816D touch(TP_I2C_SDA_PIN, TP_I2C_SCL_PIN, TP_RST, TP_INT);

// scanning i2c, mostly used for debugging
static void i2cScan(TwoWire &bus) {
  Serial.println(F("\n[I2C] Scanning..."));
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      Serial.printf("  - 0x%02X\n", addr);
      found++;
    }
  }
  if (!found) Serial.println("  (no devices)");
}

static void lvgl_log_cb(const char *buf) {
  Serial.print("[LVGL] ");
  Serial.println(buf);
}

// added percentage brightness to this, adapted from Elecrow sample code
static void initBacklight(uint8_t percent = 50) {
  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(SCREEN_BACKLIGHT_PIN, pwmChannel);
  ledcWrite(pwmChannel, (percent * 255) / 100);
}

static void flush_callback(lv_display_t *display, const lv_area_t *area, uint8_t *px_map) {
  const int32_t x = area->x1;
  const int32_t y = area->y1;
  const int32_t w = area->x2 - area->x1 + 1;
  const int32_t h = area->y2 - area->y1 + 1;

  // make sure any prior write has ended
  if (gfx.getStartCount() > 0) gfx.endWrite();

  gfx.startWrite();
  gfx.pushImageDMA(x, y, w, h, reinterpret_cast<const lgfx::rgb565_t *>(px_map));
  gfx.endWrite();

  lv_display_flush_ready(display);
}

static void touchpad_read(lv_indev_t *indev, lv_indev_data_t *data) {
  uint16_t x, y;
  uint8_t gesture = 0;
  bool touched = touch.getTouch(&x, &y, &gesture);

  Serial.println(touched ? "[Touch] touched" : "[Touch] not touched");

  if (touched) {
    data->state   = LV_INDEV_STATE_PRESSED;
    data->point.x = x;
    data->point.y = y;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static void lv_make_sanity() {
  lv_obj_t *label = lv_label_create(lv_screen_active());
  lv_label_set_text(label, "Hello LVGL 9.3");
  lv_obj_center(label);

  lv_obj_t *spin = lv_spinner_create(lv_screen_active());
  lv_obj_set_size(spin, 36, 36);
  lv_obj_align(spin, LV_ALIGN_BOTTOM_MID, 0, -8);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nBooting...");
  pinMode(POWER_LIGHT_PIN, OUTPUT);   
  digitalWrite(POWER_LIGHT_PIN, LOW); 
  pinMode(4, OUTPUT);           
  pinMode(12, OUTPUT);            

  pinMode(1, OUTPUT);   
  digitalWrite(1, HIGH); 
  pinMode(2, OUTPUT);    
  digitalWrite(2, HIGH); 
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  TwoWire WireTP = Wire;
  WireTP.setClock(400000);

  i2cScan(WireTP);

  gfx.init();
  gfx.initDMA();
  gfx.fillScreen(TFT_BLACK);
  

  led.begin();
  led.setBrightness(32);
  led.clear();
  led.show();

  for (int i = 0; i < LED_NUM; i++) { led.setPixelColor(i, led.Color(255, 64, 16)); }
  led.show();
  delay(250);
  led.clear(); led.show();

  initBacklight(100);

  pinMode(TP_INT, INPUT);
  pinMode(TP_RST, OUTPUT);

  touch.begin();
  Serial.println("[Touch] begin() called");

  // checking for 0x15
  Wire.beginTransmission(0x15);
  bool touch_present = (Wire.endTransmission() == 0);
  Serial.printf("[Touch] I2C 0x15 %s\n", touch_present ? "OK" : "NOT FOUND");

  lv_init();
 
  size_t buf_size = (size_t)HOR_RES * 40 /*lines*/ * 2 /*RGB565 bytes*/;
  buf1 = (uint8_t *)heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  buf2 = (uint8_t *)heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  if (!buf1 || !buf2) {
    Serial.println("[LVGL] Buffer alloc failed; falling back to single buffer");
    if (!buf1) buf1 = (uint8_t *)heap_caps_malloc(buf_size, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
  }

  lv_disp = lv_display_create(HOR_RES, VER_RES);
  lv_display_set_flush_cb(lv_disp, flush_callback);
  lv_display_set_buffers(lv_disp, buf1, buf2, buf_size, LV_DISPLAY_RENDER_MODE_PARTIAL);

  lv_indev = lv_indev_create();
  lv_indev_set_type(lv_indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(lv_indev, touchpad_read);

  lv_make_sanity();

  Serial.println("Setup done.");
}

void loop() {
  lv_timer_handler();
  delay(5);

  static uint32_t last = 0;
  if (millis() - last > 500) {
    uint16_t x, y; uint8_t g;
    if (touch.getTouch(&x, &y, &g)) {
      Serial.printf("[Touch] x=%u y=%u g=0x%02X\n", x, y, g);
    }
    last = millis();
  }
}