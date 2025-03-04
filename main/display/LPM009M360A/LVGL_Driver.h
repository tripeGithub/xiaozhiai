#pragma once
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "demos/lv_demos.h"

//#include "ST7789.h"
#include "display/LPM009M360A/JDI_LPM009M360A.h"
//#include "encoder.h"
//#include "buzzer.h"

#define LVGL_BUF_LEN  (TFT_HOR_RES * TFT_VER_RES / 10)
#define EXAMPLE_LVGL_TICK_PERIOD_MS    2

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define bitSet(value, bit) ((value) |= (1UL << (bit)))
#define bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#define bitWrite(value, bit, bitvalue) ((bitvalue) ? bitSet(value, bit) : bitClear(value, bit))

//extern lv_disp_draw_buf_t disp_buf;                                                 // contains internal graphic buffer(s) called draw buffer(s)
//extern lv_disp_drv_t disp_drv;                                                      // contains callback functions
extern lv_disp_t *disp;    
extern lv_indev_t *indev_encoder; //* 声明 键盘(编码器)的指针 在别的文件中
extern lv_indev_t *indev_keypad; //* 声明 键盘(编码器)的指针 在别的文件中

bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx);
//void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);
/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
//void example_lvgl_port_update_callback(lv_disp_drv_t *drv);
void example_increase_lvgl_tick(void *arg);

void jdi_lvgl_flush_cb(lv_display_t *drv, const lv_area_t *area, uint8_t *color_p);
void jdi_lvgl_invalidate_cb(lv_event_t *area);


void LVGL_Init(void);                     // Call this function to initialize the screen (must be called in the main function) !!!!!