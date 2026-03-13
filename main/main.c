#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_dev.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"
#define I2C_SCL_NUM GPIO_NUM_11
#define I2C_SDA_NUM GPIO_NUM_12
#define I2C_ADDR 0x78
#define I2C_SPEED 400*1000
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define LVGL_TICK_PERIOD_MS 10
#define LVGL_PALETTE_SIZE 8

static i2c_master_bus_handle_t i2c_bus_hdl=NULL;//I2C总线句柄
static esp_lcd_panel_io_handle_t lcd_io_hdl=NULL;//LCD IO接口句柄
static esp_lcd_panel_handle_t lcd_panel_hdl=NULL;//LCD面板句柄
static esp_timer_handle_t lvgl_tick_timer_hdl=NULL;//lvgl时钟定时器句柄
static uint8_t oled_buffer[OLED_WIDTH*OLED_HEIGHT/8];//OLED*显示*缓冲区,负责传入硬件(每个像素1bit色深,占用1位.数组中每个元素8位,可以存储8个像素)

//???
static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t io_hdl, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    /*入参:
    io_hdl:触发事件的IO接口句柄
    *edata:
    *user_ctx:
    */
    lv_display_t *disp = (lv_display_t *)user_ctx;//
    lv_display_flush_ready(disp);
    return false;
}

static void increase_lvgl_tick(void *arg)//相当于lvgl时钟:每隔10ms调用一次,告知lvgl时间流逝了10ms
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void lvgl_flush_cb(lv_display_t *disp,lv_area_t *area,uint8_t *px)//将lvgl绘图缓冲区数据传入oled硬件缓冲区
{
    esp_lcd_panel_handle_t lcd_panel_hdl=lv_display_get_user_data(disp);
    px+=LVGL_PALETTE_SIZE;//设置偏移量,跳过调色盘数据
    int x1=area->x1;
    int x2=area->x2;
    int y1=area->y1;
    int y2=area->y2;
    uint8_t byte[8]={0};
    uint8_t temp[8]={0};
    for (int y = 0; y < y2/8-y1/8+1; y++)
    {
        for (int x = 0; x < x2-x1+1; x++)
        {
            temp[0]=px[(y1+y*8+0)*(OLED_WIDTH/8)+(x1+x)/8];
            temp[1]=px[(y1+y*8+1)*(OLED_WIDTH/8)+(x1+x)/8];
            temp[2]=px[(y1+y*8+2)*(OLED_WIDTH/8)+(x1+x)/8];
            temp[3]=px[(y1+y*8+3)*(OLED_WIDTH/8)+(x1+x)/8];
            temp[4]=px[(y1+y*8+4)*(OLED_WIDTH/8)+(x1+x)/8];
            temp[5]=px[(y1+y*8+5)*(OLED_WIDTH/8)+(x1+x)/8];
            temp[6]=px[(y1+y*8+6)*(OLED_WIDTH/8)+(x1+x)/8];
            temp[7]=px[(y1+y*8+7)*(OLED_WIDTH/8)+(x1+x)/8];
            for (int i = 0; i < 8; i++)
            {
                temp[0]=(temp[0]&0x80)>>7;//第[(y1行+行偏移量y)*(OLED_WIDTH/8)列+(x1列+列偏移量x)/8]个字节
                temp[1]=(temp[1]&0x80)>>6;
                temp[2]=(temp[2]&0x80)>>5;
                temp[3]=(temp[3]&0x80)>>4;
                temp[4]=(temp[4]&0x80)>>3;
                temp[5]=(temp[5]&0x80)>>2;
                temp[6]=(temp[6]&0x80)>>1;
                temp[7]=(temp[7]&0x80)>>0;
                byte[i]=temp[0]|temp[1]|temp[2]|temp[3]|temp[4]|temp[5]|temp[6]|temp[7];
                //完成一个字节拼接后左移一位,方便再取第一位
                temp[0]=px[(y1+y*8+0)*(OLED_WIDTH/8)+(x1+x)/8]<<(1+i);
                temp[1]=px[(y1+y*8+1)*(OLED_WIDTH/8)+(x1+x)/8]<<(1+i);
                temp[2]=px[(y1+y*8+2)*(OLED_WIDTH/8)+(x1+x)/8]<<(1+i);
                temp[3]=px[(y1+y*8+3)*(OLED_WIDTH/8)+(x1+x)/8]<<(1+i);
                temp[4]=px[(y1+y*8+4)*(OLED_WIDTH/8)+(x1+x)/8]<<(1+i);
                temp[5]=px[(y1+y*8+5)*(OLED_WIDTH/8)+(x1+x)/8]<<(1+i);
                temp[6]=px[(y1+y*8+6)*(OLED_WIDTH/8)+(x1+x)/8]<<(1+i);
                temp[7]=px[(y1+y*8+7)*(OLED_WIDTH/8)+(x1+x)/8]<<(1+i);
            }
            oled_buffer[(y1/8+y)*OLED_WIDTH+x1+x]=byte[(x1+x)%8];//第[(y1/8行+行偏移量y)*OLED_WIDTH列+(x1列+列偏移量x)]个字节
            byte[(x1+x)%8]=0;
        }
    }
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(lcd_panel_hdl,x1,y1,x2+1,y2+1,oled_buffer));
}

void ssd1306_init()
{
    /*
    1.i2c总线
    2.i2c接口
    3.oled面板
    仅完成软件配置,未进行任何初始化命令的发送
    */
    i2c_master_bus_config_t i2c_bus_cfg={
        .clk_source=I2C_CLK_SRC_DEFAULT,//默认时钟源
        .scl_io_num=I2C_SCL_NUM,
        .sda_io_num=I2C_SDA_NUM,
        .i2c_port=0,//选择总线0
        .glitch_ignore_cnt=7,//毛刺过滤器
        .flags.enable_internal_pullup=true,//开启内部上拉
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg,&i2c_bus_hdl));//新建I2C总线

    esp_lcd_panel_io_i2c_config_t lcd_io_cfg={
        .dev_addr=I2C_ADDR,//地址
        .scl_speed_hz=I2C_SPEED,//时钟频率
        .control_phase_bytes=1,//写数据或命令前,先发送的控制字节数
        .dc_bit_offset=6,//D/C#在第几位
        .lcd_param_bits=8,//参数位宽
        .lcd_cmd_bits=8,//控制位宽
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus_hdl,&lcd_io_cfg,&lcd_io_hdl));//新建IO接口

    esp_lcd_panel_dev_config_t lcd_panel_cfg={
        .bits_per_pixel=1,//色深1bit
        .reset_gpio_num=-1,//无复位引脚
    };
    esp_lcd_panel_ssd1306_config_t ssd1306_cfg={
        .height=OLED_HEIGHT,//高度64像素
    };
    lcd_panel_cfg.vendor_config=&ssd1306_cfg;//供应商参数
    ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(lcd_io_hdl,&lcd_panel_cfg,&lcd_panel_hdl));//新建面板
}

void lvgl_init()
{
    /*
    1.lvgl初始化--创建显示对象,将显示对象与面板绑定
    2.分配绘图缓冲区--lvgl绘图后传入硬件缓冲区
    3.注册刷新回调函数--
    4.创建定时器--作为lvgl时钟
    */
    lv_init();
    lv_display_t *display=lv_display_create(OLED_WIDTH,OLED_HEIGHT);//创建显示对象
    lv_display_set_user_data(display,lcd_panel_hdl);//绑定显示对象和面板

    size_t draw_buffer_size = OLED_WIDTH * OLED_HEIGHT / 8 + LVGL_PALETTE_SIZE;//lvgl*绘图*缓冲区,按照lvgl要求存储像素(像素数+8*8位调色盘)
    void *buffer=NULL;//lvgl绘图缓冲区指针
    buffer=heap_caps_calloc(1,draw_buffer_size,MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);//分配缓冲区内存
    assert(buffer);//检查内存分配是否成功
    lv_display_set_color_format(display,LV_COLOR_FORMAT_I1);//设置显示色深1bit
    lv_display_set_buffers(display,buffer,NULL,draw_buffer_size,LV_DISPLAY_RENDER_MODE_FULL);//初始化绘图缓冲区
    lv_display_set_flush_cb(display,lvgl_flush_cb);//注册刷新回调函数,当lvgl完成绘图后,调用回调函数将数据传入oled硬件缓冲区

    const esp_lcd_panel_io_callbacks_t lcd_io_cb={
        .on_color_trans_done=notify_lvgl_flush_ready,//当lcd_io准备好接收下一帧数据时,调用该回调函数通知lvgl刷新完成,可以进行下一帧绘制
    };
    esp_lcd_panel_io_register_event_callbacks(lcd_io_hdl,&lcd_io_cb,display);//注册lcd_io事件回调函数

    const esp_timer_create_args_t lvgl_tick_timer_args={
        .callback=&increase_lvgl_tick,//回调函数,每隔10ms调用一次
        .name="lvgl_tick",//定时器名称
    };
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args,&lvgl_tick_timer_hdl));//创建定时器
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer_hdl,LVGL_TICK_PERIOD_MS*1000));//启动定时器,每隔10ms触发一次
}

void app_main(void)
{
    /*
    1.软件初始化--硬件未收到任何数据
    2.硬件初始化--发送初始化命令,点亮屏幕
    3.lvgl初始化--准备绘图
    */
    ssd1306_init();//软件初始化
    ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_panel_hdl));//复位面板*初始化前必须先进行复位*
    ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_panel_hdl));//面板(硬件)初始化
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_panel_hdl,true));//点亮面板
    lvgl_init();//lvgl初始化
}
