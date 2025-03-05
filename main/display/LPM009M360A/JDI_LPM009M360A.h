#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include "driver/ledc.h"

#include "Vernon_LPM009M360A.h"
//#include "CST328.h"
#include "LVGL_Driver.h"

#define SPI_MODE0   0

#define JDI_LCD_SPI_HOST             SPI2_HOST

//设置LCD SPI引脚
#define JDI_LCD_DISP_PIN        3//14//3//15
#define JDI_LCD_CS_PIN          13//4//16

#define JDI_LCD_SPI_CLK_PIN     12//5//17
#define JDI_LCD_SPI_MOSI_PIN    11//2//18

#define JDI_LCD_BLK_PIN         1

#define SPI_MISO_PIN            -1
#define SPI_HD_PIN              -1
#define SPI_WP_PIN              -1

#define SPI_PIN_NC              -1

#define JDI_LCD_SPI_PIXEL_CLOCK_HZ     1000000//(80 * 1000 * 1000)

#define TFT_HOR_RES 72      //水平
#define TFT_VER_RES 144     //垂直

#define Offset_X 0
#define Offset_Y 0

#define TFT_DRAW_BUF_HEIGHT 20

#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 1 * (LV_COLOR_DEPTH / 8))

//#define DRAW_BUF_SIZE (TFT_HOR_RES * TFT_VER_RES / 10)

#define HALF_WIDTH          (DISPLAY_WIDTH / 2)

#define COLOR_BLACK         0x00
#define COLOR_BLUE          0x02
#define COLOR_GREEN         0x04
#define COLOR_CYAN          0x06
#define COLOR_RED           0x08
#define COLOR_MAGENTA       0x0a
#define COLOR_YELLOW        0x0c
#define COLOR_WHITE         0x0e

#define CMD_NO_UPDATE           0x00
#define CMD_BLINKING_BLACK      0x10
#define CMD_BLINKING_INVERSION  0x14
#define CMD_BLINKING_WHITE      0x18
#define CMD_ALL_CLEAR           0x20
#define CMD_VCOM                0x40
#define CMD_UPDATE              0x90

typedef struct _jdi_lcd_data_t{

    bool DMA_Enabled;     // DMA启用状态的标志 Flag for DMA enabled status
    uint8_t spiBusyCheck; // 要检查的ESP32传输缓冲区的数量 The number of ESP32 transmission buffers to be checked

} jdi_lcd_data_t ;

extern jdi_lcd_data_t GVarStruct_JDI_LCD_Data;

extern esp_lcd_panel_handle_t panel_handle; // LCD面板句柄

void JDI_LCD_DISP_PIN_Config(void);
void JDI_LCD_DISP_ON(void);
void JDI_LCD_DISP_OFF(void);

void JDI_LCD_SPI_Pixel_DMA_Send(uint8_t *image, uint32_t len);

void JDI_LCD_Init(void);
void JDI_LCD_Date_Init();
void JDI_LCD_Clear_Screen(uint8_t jdi_cmd);

void LCD_Init(void);                     // Call this function to initialize the screen (must be called in the main function) !!!!!

