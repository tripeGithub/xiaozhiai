#include "LVGL_Driver.h"

//static const char *TAG_LVGL = "LVGL";

void *buf1 = NULL;
void *buf2 = NULL;
// static lv_color_t buf1[ LVGL_BUF_LEN ];
// static lv_color_t buf2[ LVGL_BUF_LEN];
    

//lv_disp_draw_buf_t disp_buf;                                                 // contains internal graphic buffer(s) called draw buffer(s)
//lv_disp_drv_t disp_drv;                                                      // contains callback functions
//lv_indev_drv_t indev_drv;
lv_indev_t *indev_encoder;  //编码器和按键的指针变量,设置group要用到
lv_indev_t *indev_keypad;  //编码器和按键的指针变量,设置group要用到

/*
void example_increase_lvgl_tick(void *arg)
{
    // Tell LVGL how many milliseconds has elapsed
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}
*/

/*
bool example_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}
*/

/*
void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1 + Offset_X, offsety1 + Offset_Y, offsetx2 + Offset_X + 1, offsety2 + Offset_Y + 1, color_map);
}
*/

/*
//Read the touchpad
void example_touchpad_read( lv_indev_drv_t * drv, lv_indev_data_t * data )
{
    uint16_t touchpad_x[5] = {0};
    uint16_t touchpad_y[5] = {0};
    uint8_t touchpad_cnt = 0;

    // Read touch controller data 
    esp_lcd_touch_read_data(drv->user_data);

    // Get coordinates 
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 5);

    // printf("CCCCCCCCCCCCC=%d  \r\n",touchpad_cnt);
    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PR;
        // printf("X=%u Y=%u num=%d \r\n", data->point.x, data->point.y,touchpad_cnt);
    } else {
        data->state = LV_INDEV_STATE_REL;
    } 
}
*/

/*
// Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated.
void example_lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);
        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);
        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);
        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);
        break;
    }
}
*/

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// lv_disp_t *disp;
// void LVGL_Init(void)
// {
//     ESP_LOGI(TAG_LVGL, "Initialize LVGL library");
//     lv_init();
    
//     buf1 = heap_caps_malloc(DRAW_BUF_SIZE/2 , MALLOC_CAP_SPIRAM);
//     assert(buf1);
//     buf2 = heap_caps_malloc(DRAW_BUF_SIZE/2 , MALLOC_CAP_SPIRAM);    
//     assert(buf2);
//     lv_disp_draw_buf_init(&disp_buf, buf1, buf2, DRAW_BUF_SIZE/2);      // initialize LVGL draw buffers
//     //lv_disp_draw_buf_init(&disp_buf, buf1, NULL, DRAW_BUF_SIZE/2);

//     /*
//     buf1 = heap_caps_malloc(LVGL_BUF_LEN , MALLOC_CAP_SPIRAM);
//     assert(buf1);
//     buf2 = heap_caps_malloc(LVGL_BUF_LEN , MALLOC_CAP_SPIRAM);    
//     assert(buf2);
//     lv_disp_draw_buf_init(&disp_buf, buf1, buf2, LVGL_BUF_LEN);         // initialize LVGL draw buffers
//     */

//     ESP_LOGI(TAG_LVGL, "Register display driver to LVGL");
//     lv_disp_drv_init(&disp_drv);                                        // Create a new screen object and initialize the associated device
//     disp_drv.hor_res = TFT_HOR_RES;             
//     disp_drv.ver_res = TFT_VER_RES;                                     // Horizontal pixel count
//     disp_drv.rotated = LV_DISP_ROT_NONE; // 图像旋转                    // Vertical axis pixel count
//     //disp_drv.flush_cb = example_lvgl_flush_cb;                        // Function : copy a buffer's content to a specific area of the display
//     disp_drv.flush_cb = jdi_lvgl_flush_cb;
//     //disp_drv.rounder_cb = jdi_lvgl_invalidate_cb;
//     disp_drv.drv_update_cb = example_lvgl_port_update_callback;         // Function : Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. 
//     disp_drv.draw_buf = &disp_buf;                                      // LVGL will use this buffer(s) to draw the screens contents
//     disp_drv.user_data = panel_handle;                
//     ESP_LOGI(TAG_LVGL,"Register display indev to LVGL");                // Custom display driver user data
//     disp = lv_disp_drv_register(&disp_drv);    

//     //触摸配置 
//     /*
//     lv_indev_drv_init ( &indev_drv );
//     indev_drv.type = LV_INDEV_TYPE_POINTER;
//     indev_drv.disp = disp;
//     indev_drv.read_cb = example_touchpad_read;
//     indev_drv.user_data = tp;
//     lv_indev_drv_register( &indev_drv );
//     */

//     //编码器配置
//     //*Initialize the input device*/
//     static lv_indev_drv_t encoder_drv;                   // 创建结构体变量
//     lv_indev_drv_init(&encoder_drv);                     // 初始化 成员赋初值
//     encoder_drv.type = LV_INDEV_TYPE_ENCODER;            // 编码器模式驱动按键更好用
//     encoder_drv.read_cb = encoder_read;                  // 回调函数：通过这个函数读取输入设备的值 
//     indev_encoder = lv_indev_drv_register(&encoder_drv); // 返回的指针保留为全局变量设置group要用到

//     //按键配置
//     //*Initialize the input device*/
//     /*
//     static lv_indev_drv_t keypad_drv;                   // 创建结构体变量
//     lv_indev_drv_init(&keypad_drv);                     // 初始化 成员赋初值
//     keypad_drv.type = LV_INDEV_TYPE_KEYPAD;            // 编码器模式驱动按键更好用
//     keypad_drv.read_cb = encoder_read;                  // 回调函数：通过这个函数读取输入设备的值 
//     indev_keypad = lv_indev_drv_register(&keypad_drv); // 返回的指针保留为全局变量设置group要用到
//     */

//     /********************* LVGL *********************/
//     ESP_LOGI(TAG_LVGL, "Install LVGL tick timer");
//     // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
//     const esp_timer_create_args_t lvgl_tick_timer_args = {
//         .callback = &example_increase_lvgl_tick,
//         .name = "lvgl_tick"
//     };
    
//     esp_timer_handle_t lvgl_tick_timer = NULL;
//     ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
//     ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

// }


///////////////////////////////////////////////////////////////////////////////////////////////////

bool dis_shake = false; // 是否开启抖动
//bool dis_shake = true;

volatile uint32_t flush_count = 0;

// 创建颜色
///*
// r:0-31,g:0-63,b:0-31
uint16_t create_rgb565(uint8_t r, uint8_t g, uint8_t b)
{
  return ((r << 11) | (g << 5) | b);
}
//*/

/*
uint16_t create_rgb565(float f_r, float f_g, float f_b)
{
  uint8_t r;
  uint8_t g;
  uint8_t b;

  if (f_r > 0) r = f_r + 0.5;
  else         r = 0;
  if (f_g > 0) g = f_g + 0.5;
  else         g = 0;
  if (f_b > 0) b = f_b + 0.5;
  else         b = 0;
  
  return ((r << 11) | (g << 5) | b);
}
*/

/*
void modify_pixel_color(uint8_t *color_p, int16_t x, int16_t y, uint16_t new_color)
{
  //printf("color_p address1: %p\n", (void *)color_p);
  int32_t pixel_offset = (y * TFT_HOR_RES + x) * 2; // 计算要修改像素在缓冲区中的偏移量
  uint8_t high_byte = (new_color >> 8) & 0xFF;      // 获取高8位
  uint8_t low_byte = new_color & 0xFF;              // 获取低8位
  uint8_t *pixel_addr = color_p + pixel_offset;     // 获取要修改像素颜色值的内存地址
  // 修改像素颜色值
  *pixel_addr = low_byte;
  pixel_addr++;
  *pixel_addr = high_byte;
}
*/

///*
void modify_pixel_color(uint8_t *color_p, uint32_t pixel_offset, uint16_t new_color)
{
  //printf("color_p address1: %p\n", (void *)color_p);
  uint8_t high_byte = (new_color >> 8) & 0xFF;      // 获取高8位
  uint8_t low_byte = new_color & 0xFF;              // 获取低8位
  uint8_t *pixel_addr = color_p + pixel_offset;     // 获取要修改像素颜色值的内存地址
  // 修改像素颜色值
  *pixel_addr = low_byte;
  pixel_addr++;
  *pixel_addr = high_byte;
}
//*/

uint32_t getPixelIdx(int32_t x, int32_t y)
{
  return (y * TFT_HOR_RES + x) * 2;
}

//val1附近的像素，val2误差
uint8_t colorThresholdLimit(uint8_t val1, int8_t val2,uint8_t max) // 颜色阈值限制
{
  int16_t val1_int = val1;
  int16_t val2_int = val2;
  int16_t tmp = val1_int + val2_int;
  //Serial.print("val1_int:" + String(val1_int)); Serial.print(" val2_int:" + String(val2_int)); Serial.println(" tmp:" + String(tmp));
  if (tmp > max) return max;
  else if (tmp < 0) return 0;
  else return tmp;
  return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////

// LVGL渲染函数，使用pushPixelsDMA绘制，速度快，暂未实现旋转
//void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
//void jdi_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, uint8_t *color_p)
void jdi_lvgl_flush_cb(lv_display_t *drv, const lv_area_t *area, uint8_t *color_p)
{
    //uint8_t *color_p = (uint8_t *)lv_color_p;
    
    /*Serial.println();
    Serial.print("area->x1:");Serial.println(area->x1);
    Serial.print("area->x2:");Serial.println(area->x2);
    Serial.print("area->y1:");Serial.println(area->y1);
    Serial.print("area->y2:");Serial.println(area->y2);
    Serial.println();
    delay(1000);*/

    // DMA发送缓冲区
    // 1个字节存储两个像素的颜色所以只需要屏幕的一半
    // 头指令2个+末尾结束指令2个所以+4
    // 因为DMA有BUG，需要发送多4个字节才能正常
    //uint16_t BUF_HOR_NUM = TFT_HOR_RES / 2 + 4 + 4;
    uint16_t BUF_HOR_NUM = TFT_HOR_RES / 2 + 4;
    uint8_t buf_HOR[BUF_HOR_NUM];
    memset(&buf_HOR, 0, sizeof(buf_HOR));

    // 当前颜色
    uint16_t color_cur;          // 当前像素的颜色
    uint8_t r_cur, g_cur, b_cur; // 旧的颜色 量化的颜色
    int8_t r_err, g_err, b_err;  // 颜色误差

    // 像素抖动参数
    uint32_t pixel_offset_d;
    uint8_t *pixel_d;
    uint16_t color_d;
    uint8_t r_d;
    uint8_t g_d;
    uint8_t b_d;

    for (lv_coord_t y = area->y1; y <= area->y2; y++) // area->y1 area->y2
    {
        //Serial.print(y);Serial.print(": ");

        // 行开头添加开始命令
        buf_HOR[0] = CMD_UPDATE;
        buf_HOR[1] = y + 1;

        for (lv_coord_t x = area->x1; x <= area->x2; x++) //area->x1 area->x2
        {
            // 获取当前地址的颜色，从rgb565拆成rgb分量
            color_cur = color_p[1] << 8 | color_p[0];
            //r_cur, g_cur, b_cur; // 旧的颜色 量化的颜色
            //r_err, g_err, b_err;  // 颜色误差
            r_cur = (color_cur >> 11) & 0x1F;
            g_cur = (color_cur >> 5) & 0x3F;
            b_cur = color_cur & 0x1F;

            // 量化颜色 将16位颜色转换为8位
            if (r_cur > 31 / 2) {r_err = r_cur - 31; r_cur = 31;}
            else                {r_err = r_cur - 0;  r_cur = 0;}
            if (g_cur > 63 / 2) {g_err = g_cur - 63; g_cur = 63;}
            else                {g_err = g_cur - 0;  g_cur = 0;}
            if (b_cur > 31 / 2) {b_err = b_cur - 31; b_cur = 31;}
            else                {b_err = b_cur - 0;  b_cur = 0;}

            if (dis_shake) // 抖动开启
            {
                modify_pixel_color(color_p, 0, create_rgb565(r_cur, g_cur, b_cur));

                // 进行误差扩散
                if (x != TFT_HOR_RES - 1)
                {
                    //pixel_offset_d = getPixelIdx(1, 0);
                    pixel_offset_d = 2;
                    pixel_d = color_p + pixel_offset_d;
                    color_d = pixel_d[1] << 8 | pixel_d[0];
                    r_d = (color_d >> 11) & 0x1F;
                    g_d = (color_d >> 5) & 0x3F;
                    b_d = color_d & 0x1F;
                    r_d = colorThresholdLimit(r_d, (r_err * 7) / 16, 31);
                    g_d = colorThresholdLimit(g_d, (g_err * 7) / 16, 63);
                    b_d = colorThresholdLimit(b_d, (b_err * 7) / 16, 31);
                    color_d = create_rgb565(r_d, g_d, b_d);
                    modify_pixel_color(color_p, pixel_offset_d, color_d);
                }
                if (x != 0 && y != TFT_VER_RES - 1)
                {
                    //pixel_offset_d = getPixelIdx(-1, 1);
                    pixel_offset_d = 142;
                    pixel_d = color_p + pixel_offset_d;
                    color_d = pixel_d[1] << 8 | pixel_d[0];
                    r_d = (color_d >> 11) & 0x1F;
                    g_d = (color_d >> 5) & 0x3F;
                    b_d = color_d & 0x1F;
                    r_d = colorThresholdLimit(r_d, (r_err * 5) / 16, 31);
                    g_d = colorThresholdLimit(g_d, (g_err * 5) / 16, 63);
                    b_d = colorThresholdLimit(b_d, (b_err * 5) / 16, 31);
                    color_d = create_rgb565(r_d, g_d, b_d);
                    modify_pixel_color(color_p, pixel_offset_d, color_d);
                }
                if (y != TFT_VER_RES - 1)
                {
                    //pixel_offset_d = getPixelIdx(0, 1);
                    pixel_offset_d = 144;
                    pixel_d = color_p + pixel_offset_d;
                    color_d = pixel_d[1] << 8 | pixel_d[0];
                    r_d = (color_d >> 11) & 0x1F;
                    g_d = (color_d >> 5) & 0x3F;
                    b_d = color_d & 0x1F;
                    r_d = colorThresholdLimit(r_d, (r_err * 3) / 16, 31);
                    g_d = colorThresholdLimit(g_d, (g_err * 3) / 16, 63);
                    b_d = colorThresholdLimit(b_d, (b_err * 3) / 16, 31);
                    color_d = create_rgb565(r_d, g_d, b_d);
                    modify_pixel_color(color_p, pixel_offset_d, color_d);
                }
                if (x != TFT_HOR_RES - 1 && y != TFT_VER_RES - 1)
                {
                    //pixel_offset_d = getPixelIdx(1, 1);
                    pixel_offset_d = 146;
                    pixel_d = color_p + pixel_offset_d;
                    color_d = pixel_d[1] << 8 | pixel_d[0];
                    r_d = (color_d >> 11) & 0x1F;
                    g_d = (color_d >> 5) & 0x3F;
                    b_d = color_d & 0x1F;
                    r_d = colorThresholdLimit(r_d, (r_err * 1) / 16, 31);
                    g_d = colorThresholdLimit(g_d, (g_err * 1) / 16, 63);
                    b_d = colorThresholdLimit(b_d, (b_err * 1) / 16, 31);
                    color_d = create_rgb565(r_d, g_d, b_d);
                    modify_pixel_color(color_p, pixel_offset_d, color_d);
                }
            }
            
            uint8_t jdi_color = 0;
            if(r_cur == 31) bitWrite(jdi_color, 3, 1);
            if(g_cur == 63) bitWrite(jdi_color, 2, 1);
            if(b_cur == 31) bitWrite(jdi_color, 1, 1);

            color_p += 2; //16位颜色占用两个字节，移动到下一个地址

            // 一个字节存储两个像素的颜色
            int pixelIdx1 = (x / 2);
            if (x % 2 == 0)
            {
                buf_HOR[pixelIdx1 + 2] &= 0x0F;
                buf_HOR[pixelIdx1 + 2] |= (jdi_color & 0x0F) << 4;
            }
            else
            {
                buf_HOR[pixelIdx1 + 2] &= 0xF0;
                buf_HOR[pixelIdx1 + 2] |= jdi_color & 0x0F;
            }

            //Serial.print(draw_buf[x + y * TFT_HOR_RES]);
            //Serial.print(" ");
        }

        // 行结尾添加结束命令
        buf_HOR[area->x2 + 3] = 0x00;
        buf_HOR[area->x2 + 4] = 0x00;
        buf_HOR[area->x2 + 5] = 0x00;
        buf_HOR[area->x2 + 6] = 0x00;
        buf_HOR[area->x2 + 7] = 0x00;
        buf_HOR[area->x2 + 8] = 0x00;

        //推送一行
        JDI_LCD_SPI_Pixel_DMA_Send(buf_HOR, BUF_HOR_NUM);

        //delay(500);
        //Serial.println();
    }

    //flush_count++;

    // delay(1000);
    lv_disp_flush_ready(drv);
}

void jdi_lvgl_invalidate_cb(lv_event_t *area)
{
    // 获取发送事件时传递的参数
  lv_area_t *e = (lv_area_t *)lv_event_get_param(area);
  e->x1 = 0;
  e->x2 = TFT_HOR_RES - 1;
}
