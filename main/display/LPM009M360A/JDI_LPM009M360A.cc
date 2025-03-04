#include "JDI_LPM009M360A.h"

static const char *TAG_LCD = "JDI_LPM009M360A";

//配置输出寄存器
#define JDI_LCD_DISP_PIN_SEL    (1ULL<<JDI_LCD_DISP_PIN)
#define JDI_LCD_CS_PIN_SEL      (1ULL<<JDI_LCD_CS_PIN)
#define JDI_LCD_BLK_PIN_SEL      (1ULL<<JDI_LCD_BLK_PIN)

spi_device_handle_t jdi_lcd_spi_port;
spi_device_handle_t *p_jdi_lcd_spi_port;

jdi_lcd_data_t GVarStruct_JDI_LCD_Data;

extern volatile spi_device_handle_t *spi_dev;


void JDI_LCD_Date_Init(void)
{
    //GVarStruct_JDI_LCD_Data.DMA_Enabled = false;
    GVarStruct_JDI_LCD_Data.spiBusyCheck = 0;

    GVarStruct_JDI_LCD_Data.DMA_Enabled = true;

}

/**
 * @函数说明        LED的初始化
 *
 */
void JDI_LCD_DISP_PIN_Config(void)
{
    gpio_config_t gpio_lcd_disp_init_struct = {0};
    //gpio_config_t gpio_lcd_cs_init_struct = {0};
    gpio_config_t gpio_lcd_blk_init_struct = {0};

    //DISP PIN
    //配置IO为通用IO
    esp_rom_gpio_pad_select_gpio(JDI_LCD_DISP_PIN);

    gpio_lcd_disp_init_struct.intr_type = GPIO_INTR_DISABLE;             //不使用中断
    gpio_lcd_disp_init_struct.mode = GPIO_MODE_OUTPUT;                   //输出模式
    gpio_lcd_disp_init_struct.pull_up_en = GPIO_PULLUP_ENABLE;           //使能上拉模式
    gpio_lcd_disp_init_struct.pull_down_en = GPIO_PULLDOWN_DISABLE;      //失能下拉模式
    gpio_lcd_disp_init_struct.pin_bit_mask = JDI_LCD_DISP_PIN_SEL;        //使用GPIO9输出寄存器

    //将以上参数配置到引脚
    gpio_config( &gpio_lcd_disp_init_struct );

    //设置引脚输出低电平，默认不让LCD刷新
    gpio_set_level((gpio_num_t)JDI_LCD_DISP_PIN, 0);

    //--------------------------------------------------------------------------------------------
    /*
    //CS PIN
    //
    //配置IO为通用IO
    esp_rom_gpio_pad_select_gpio(JDI_LCD_CS_PIN);

    gpio_lcd_cs_init_struct.intr_type = GPIO_INTR_DISABLE;             //不使用中断
    gpio_lcd_cs_init_struct.mode = GPIO_MODE_OUTPUT;                   //输出模式
    gpio_lcd_cs_init_struct.pull_up_en = GPIO_PULLUP_ENABLE;           //使能上拉模式
    gpio_lcd_cs_init_struct.pull_down_en = GPIO_PULLDOWN_DISABLE;      //失能下拉模式
    gpio_lcd_cs_init_struct.pin_bit_mask = JDI_LCD_CS_PIN_SEL;         //使用GPIO9输出寄存器

    //将以上参数配置到引脚
    gpio_config( &gpio_lcd_cs_init_struct );

    //设置CS输出低电平
    gpio_set_level(JDI_LCD_CS_PIN, 0);
    */

    //--------------------------------------------------------------------------------------------
    ///*
    //BLK PIN
    //
    //配置IO为通用IO
    esp_rom_gpio_pad_select_gpio(JDI_LCD_BLK_PIN);

    gpio_lcd_blk_init_struct.intr_type = GPIO_INTR_DISABLE;             //不使用中断
    gpio_lcd_blk_init_struct.mode = GPIO_MODE_OUTPUT;                   //输出模式
    gpio_lcd_blk_init_struct.pull_up_en = GPIO_PULLUP_ENABLE;           //使能上拉模式
    gpio_lcd_blk_init_struct.pull_down_en = GPIO_PULLDOWN_DISABLE;      //失能下拉模式
    gpio_lcd_blk_init_struct.pin_bit_mask = JDI_LCD_BLK_PIN_SEL;         //使用GPIO9输出寄存器

    //将以上参数配置到引脚
    gpio_config( &gpio_lcd_blk_init_struct );

    //设置CS输出低电平
    gpio_set_level((gpio_num_t)JDI_LCD_BLK_PIN, 1);
    //*/
}

esp_err_t JDI_LCD_SPI_Config(void)
{
    //00 定义错误标志
	esp_err_t e;

    
	//01 配置总线初始化结构体
	static spi_bus_config_t bus_cfg;    //总线配置结构体

    bus_cfg.miso_io_num		= SPI_PIN_NC;			    //miso
	bus_cfg.mosi_io_num		= JDI_LCD_SPI_MOSI_PIN;	    //mosi
	bus_cfg.sclk_io_num		= JDI_LCD_SPI_CLK_PIN;	    //sclk
	bus_cfg.quadhd_io_num	= SPI_PIN_NC;			    // HD
	bus_cfg.quadwp_io_num	= SPI_PIN_NC;			    // WP
	bus_cfg.max_transfer_sz = 4092;					    //非DMA最大64bytes,DMA最大4092bytes
	//bus_cfg.intr_flags = 0;						    //这个用于设置中断优先级的，0是默认
	bus_cfg.flags = SPICOMMON_BUSFLAG_MASTER;
    //这个用于设置初始化的时候要检测哪些选项。比如这里设置的是spi初始化为主机模式是否成功。检测结果通过spi_bus_initialize函数的
    //返回值进行返回。如果初始化为主机模式成功，就会返回esp_ok

    //02 初始化总线配置结构体
    e = spi_bus_initialize(JDI_LCD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

    if (e != ESP_OK)
    {
        printf("bus initialize failed!\n");
        return e;
    }
    

    //03 配置设备结构体
    static spi_device_interface_config_t spi_interface_cfg; //设备配置结构体

    spi_interface_cfg.command_bits      = 0;    // 命令阶段
    spi_interface_cfg.address_bits      = 0;    // 地址阶段
    spi_interface_cfg.dummy_bits        = 0;
    spi_interface_cfg.mode              = SPI_MODE0; // 模式
    spi_interface_cfg.duty_cycle_pos    = 0;
    spi_interface_cfg.cs_ena_pretrans   = 0;
    spi_interface_cfg.cs_ena_posttrans  = 0;
    spi_interface_cfg.clock_speed_hz    = 4000000; // 频率
    spi_interface_cfg.input_delay_ns    = 0;
    spi_interface_cfg.spics_io_num      = JDI_LCD_CS_PIN;
    spi_interface_cfg.flags             = SPI_DEVICE_POSITIVE_CS; 
    spi_interface_cfg.queue_size        = 10;    // 队列数量
    spi_interface_cfg.pre_cb            = 0;    // dc_callback，//回调处理D/C行
    spi_interface_cfg.post_cb           = 0;

    
    //04 设备初始化
    e = spi_bus_add_device(JDI_LCD_SPI_HOST, &spi_interface_cfg, &jdi_lcd_spi_port);

    if (e != ESP_OK)
    {
        printf("device config error\n");
        return e;
    }
    

    GVarStruct_JDI_LCD_Data.DMA_Enabled = true; //使能SPI DMA标志

    return e;
} 

/**
 * @函数说明        设置LED亮
 *
 */
void JDI_LCD_DISP_ON(void)
{
    gpio_set_level((gpio_num_t)JDI_LCD_DISP_PIN, 1);
}


/**
 * @函数说明        设置LED灭
 *
 */
void JDI_LCD_DISP_OFF(void)
{
    gpio_set_level((gpio_num_t)JDI_LCD_DISP_PIN, 0);
}


void JDI_LCD_BLK_ON(void)
{
    gpio_set_level((gpio_num_t)JDI_LCD_BLK_PIN, 0);
}

void JDI_LCD_BLK_OFF(void)
{
    gpio_set_level((gpio_num_t)JDI_LCD_BLK_PIN, 1);
}


void spi_dma_wait(void)
{
    jdi_lcd_data_t *ptr_jdi_lcd_data;
    ptr_jdi_lcd_data = &GVarStruct_JDI_LCD_Data;
    
    if (!ptr_jdi_lcd_data->DMA_Enabled || !ptr_jdi_lcd_data->spiBusyCheck)  return;

    spi_transaction_t *rtrans;
    esp_err_t ret;

    for (int i = 0; i < ptr_jdi_lcd_data->spiBusyCheck; ++i)
    {
        //ret = spi_device_get_trans_result(jdi_lcd_spi_port, &rtrans, portMAX_DELAY);
        ret = spi_device_get_trans_result(*spi_dev, &rtrans, portMAX_DELAY);
        assert(ret == ESP_OK);
    }

    ptr_jdi_lcd_data->spiBusyCheck = 0;

}

void JDI_LCD_SPI_Pixel_DMA_Send(uint8_t *image, uint32_t len)
{
    jdi_lcd_data_t *ptr_jdi_lcd_data;
    ptr_jdi_lcd_data = &GVarStruct_JDI_LCD_Data;
    
    if ((len == 0) || (!ptr_jdi_lcd_data->DMA_Enabled)) return;

    spi_dma_wait();

    /*if (_swapBytes)
    {
      for (uint32_t i = 0; i < len; i++)
        (image[i] = image[i] << 8 | image[i] >> 8);
    }*/

    esp_err_t ret;
    static spi_transaction_t trans;

    memset(&trans, 0, sizeof(spi_transaction_t));

    trans.user = (void *)1;
    trans.tx_buffer = image; // 数据指针
    trans.length = len * 8;  // 数据长度，以位为单位 uint8_t = 8bit uint16_t = 16bit
    trans.flags = 0;         // SPI_TRANS_USE_TXDATA标志
    trans.rx_buffer = NULL;  // 指向接收缓冲区的指针，或NULL表示无MISO阶段。如果使用DMA，则以4字节为单位写入。

    //ret = spi_device_queue_trans(jdi_lcd_spi_port, &trans, portMAX_DELAY);
    ret = spi_device_queue_trans(*spi_dev, &trans, portMAX_DELAY);
    assert(ret == ESP_OK);
    // Serial.print("ret:"); Serial.println(ret);
    ptr_jdi_lcd_data->spiBusyCheck++;
}

void JDI_LCD_Clear_Screen(uint8_t jdi_cmd)
{
    jdi_lcd_data_t *ptr_jdi_lcd_data;
    ptr_jdi_lcd_data = &GVarStruct_JDI_LCD_Data;
    
    if (ptr_jdi_lcd_data->DMA_Enabled)
    {
        uint8_t buf[2];
        //buf[0] = CMD_BLINKING_WHITE;
        buf[0] = jdi_cmd;
        buf[1] = 0x00;
        JDI_LCD_SPI_Pixel_DMA_Send(buf, 2);
    }
    else
    {
        /*
        digitalWrite(_scs, HIGH);
        SPI.transfer(CMD_ALL_CLEAR);
        SPI.transfer(0x00);
        digitalWrite(_scs, LOW);
        */
    }
}


void JDI_LCD_Init(void)
{
    JDI_LCD_DISP_PIN_Config();

    //JDI_LCD_DISP_OFF();
    
    JDI_LCD_SPI_Config();

    JDI_LCD_DISP_ON();

    JDI_LCD_Clear_Screen(CMD_BLINKING_WHITE);
}

///////////////////////////////////////////

esp_lcd_panel_handle_t panel_handle = NULL;

void LCD_Init(void)
{
    ESP_LOGI(TAG_LCD, "Initialize SPI bus"); 
    
    // spi_bus_config_t buscfg = {                                                         
    //     .sclk_io_num = JDI_LCD_SPI_CLK_PIN,                                            
    //     .mosi_io_num = JDI_LCD_SPI_MOSI_PIN,                                            
    //     .miso_io_num = -1,                                            
    //     .quadwp_io_num = -1,                                                            
    //     .quadhd_io_num = -1,                                                            
    //     .max_transfer_sz = 4092, 
    //     .flags = SPICOMMON_BUSFLAG_MASTER,   
    // };
    // ESP_ERROR_CHECK(spi_bus_initialize(JDI_LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO)); 
    // ESP_LOGI(TAG_LCD, "Install panel IO");   

    // esp_lcd_panel_io_handle_t io_handle = NULL;                                         
    // esp_lcd_panel_io_spi_config_t io_config = {                                             
    //     .dc_gpio_num = -1,
    //     .cs_gpio_num = JDI_LCD_CS_PIN,
    //     .pclk_hz = JDI_LCD_SPI_PIXEL_CLOCK_HZ,
    //     .lcd_cmd_bits = 0,
    //     .lcd_param_bits = 0,
    //     .spi_mode = SPI_MODE0,
    //     .flags.cs_high_active = 1,
    //     .trans_queue_depth = 3,
    //     .on_color_trans_done = example_notify_lvgl_flush_ready,
    //     .user_ctx = &disp_drv,
    // };
    // // Attach the LCD to the SPI bus
    // ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi_user((esp_lcd_spi_bus_handle_t)JDI_LCD_SPI_HOST, &io_config, &io_handle));
    
    // esp_lcd_panel_dev_LPM009M360A_config_t panel_config = {
    //     .reset_gpio_num = -1,
    //     .rgb_endian = LCD_RGB_ENDIAN_RGB,
    //     .bits_per_pixel = 16,
    // };
    // ESP_LOGI(TAG_LCD, "Install LPM009M360A panel driver");
    // ESP_ERROR_CHECK(esp_lcd_new_panel_LPM009M360A(io_handle, &panel_config, &panel_handle));
    
    /*
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, false));
    */

    /*
    // user can flush pre-defined pattern to the screen before we turn on the screen or backlight
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    */

    //ESP_LOGI(TAG_LCD, "Turn on LCD backlight");
    // gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
    
    //Backlight_Init();    
    //TOUCH_Init();

    ///*
    JDI_LCD_Date_Init();

    JDI_LCD_DISP_PIN_Config();

    //JDI_LCD_BLK_ON();
    JDI_LCD_BLK_OFF();

    JDI_LCD_DISP_ON();

    //JDI_LCD_Clear_Screen(CMD_BLINKING_WHITE);
    //*/
}
