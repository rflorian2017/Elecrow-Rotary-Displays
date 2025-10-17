#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_USE_CONFIG 1

#define LV_COLOR_DEPTH 16

#define LV_USE_LOG 1
#define LV_LOG_LEVEL LV_LOG_LEVEL_INFO

#define LV_USE_ASSERT_NULL 1
#define LV_USE_ASSERT_MALLOC 1
#define LV_USE_ASSERT_STYLE 1

#define LV_TICK_CUSTOM 1
#if LV_TICK_CUSTOM
  #define LV_TICK_CUSTOM_INCLUDE <Arduino.h>
  #define LV_TICK_CUSTOM_SYS_TIME_EXPR (millis())
#endif

#endif