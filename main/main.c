#include <stdio.h>
#include <unistd.h>
#include <assert.h>
#include <sys/lock.h>
#include <sys/param.h>
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
#define I2C_SCL_NUM GPIO_NUM_9
#define I2C_SDA_NUM GPIO_NUM_10
#define I2C_ADDR 0x3C 
#define I2C_SPEED (100*1000)
#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define LVGL_TICK_PERIOD_MS 5
#define LVGL_TASK_DELAY_MS_MAX 500
#define LVGL_TASK_DELAY_MS_MIN 1000/CONFIG_FREERTOS_HZ
#define LVGL_PALETTE_SIZE 8

static i2c_master_bus_handle_t i2c_bus_hdl=NULL;//I2C总线句柄
static esp_lcd_panel_io_handle_t lcd_io_hdl=NULL;//LCD IO接口句柄
static esp_lcd_panel_handle_t lcd_panel_hdl=NULL;//LCD面板句柄
static esp_timer_handle_t lvgl_tick_timer_hdl=NULL;//lvgl时钟定时器句柄
static lv_display_t *display=NULL;//lvgl显示对象句柄
static uint8_t oled_buffer_fullscreen[OLED_WIDTH*OLED_HEIGHT/8];//OLED*显示*缓冲区,负责传入硬件(每个像素1bit色深,占用1位.数组中每个元素8位,可以存储8个像素)
static _lock_t lvgl_api_lock;//互斥锁,保护lvgl线程安全

static void ssd1306_init()
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
        .flags.enable_internal_pullup=false,//关闭内部上拉
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

static void increase_lvgl_tick()//相当于lvgl时钟:每隔10ms调用一次,告知lvgl时间流逝了10ms
{
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

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

static void lvgl_flush_cb(lv_display_t *disp,const lv_area_t *area,uint8_t *px)//将lvgl绘图缓冲区数据传入oled硬件缓冲区
{
    lcd_panel_hdl=lv_display_get_user_data(disp);
    px+=LVGL_PALETTE_SIZE;//设置偏移量,跳过调色盘数据
    int x1=area->x1;
    int x2=area->x2;
    int y1_fullpage=area->y1&0xF8;//清零低3位取整到整页
    int y2_fullpage=(area->y2&0xF8)+7;//清零低三位再＋7取整到整页
    uint8_t temp=0;//临时存1页像素
    uint8_t page=0;//页号
    //全局刷新模式
    for (int y = y1_fullpage; y <= y2_fullpage; y+=8)//按页遍历刷新区域,每次处理8行像素
    {
        page=y>>3;//右移3位得到当前页号
        for (int x = x1; x <= x2; x++)//逐列取出当前页对应的8个垂直像素
        {
            temp=0;//清零临时字节,准备重新打包这一列在当前页中的8个像素
            uint8_t mask=0x80>>(x%8);//当前列在lvgl源缓冲区对应字节中的位掩码***非常天才的方法***
            if (px[(y*OLED_WIDTH>>3)+(x>>3)]&mask)     temp|=0x01;//取第1行像素,放到页字节最低位
            if (px[((y+1)*OLED_WIDTH>>3)+(x>>3)]&mask) temp|=0x02;//取第2行像素,放到页字节第1位
            if (px[((y+2)*OLED_WIDTH>>3)+(x>>3)]&mask) temp|=0x04;//取第3行像素,放到页字节第2位
            if (px[((y+3)*OLED_WIDTH>>3)+(x>>3)]&mask) temp|=0x08;//取第4行像素,放到页字节第3位
            if (px[((y+4)*OLED_WIDTH>>3)+(x>>3)]&mask) temp|=0x10;//取第5行像素,放到页字节第4位
            if (px[((y+5)*OLED_WIDTH>>3)+(x>>3)]&mask) temp|=0x20;//取第6行像素,放到页字节第5位
            if (px[((y+6)*OLED_WIDTH>>3)+(x>>3)]&mask) temp|=0x40;//取第7行像素,放到页字节第6位
            if (px[((y+7)*OLED_WIDTH>>3)+(x>>3)]&mask) temp|=0x80;//取第8行像素,放到页字节最高位
            oled_buffer_fullscreen[page*OLED_WIDTH+x]=temp;//写入oled页缓冲区中当前页的当前列
        }
    }
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(lcd_panel_hdl,x1,y1_fullpage,x2+1,y2_fullpage+1,oled_buffer_fullscreen));
    /*局部刷新模式
    int width=x2-x1+1;//刷新区域宽度
    static uint8_t oled_buffer_partial[OLED_WIDTH * OLED_HEIGHT / 8 / 10];//局部刷新缓冲区
    page=0;//从缓冲区第0页开始
    for (int y = y1_fullpage; y <= y2_fullpage; y+=8)//按页遍历刷新区域,每次处理8行像素
    {
        for (int x = x1; x <= x2; x++)//逐列取出当前页对应的8个垂直像素
        {
            temp=0;//清零临时字节,准备重新打包这一列在当前页中的8个像素
            uint8_t mask=0x80>>(x%8);//当前列在lvgl源缓冲区对应字节中的位掩码***非常天才的方法***
            if (px[(y*width>>3)+(x>>3)]&mask)     temp|=0x01;//取第1行像素,放到页字节最低位
            if (px[((y+1)*width>>3)+(x>>3)]&mask) temp|=0x02;//取第2行像素,放到页字节第1位
            if (px[((y+2)*width>>3)+(x>>3)]&mask) temp|=0x04;//取第3行像素,放到页字节第2位
            if (px[((y+3)*width>>3)+(x>>3)]&mask) temp|=0x08;//取第4行像素,放到页字节第3位
            if (px[((y+4)*width>>3)+(x>>3)]&mask) temp|=0x10;//取第5行像素,放到页字节第4位
            if (px[((y+5)*width>>3)+(x>>3)]&mask) temp|=0x20;//取第6行像素,放到页字节第5位
            if (px[((y+6)*width>>3)+(x>>3)]&mask) temp|=0x40;//取第7行像素,放到页字节第6位
            if (px[((y+7)*width>>3)+(x>>3)]&mask) temp|=0x80;//取第8行像素,放到页字节最高位
            oled_buffer_partial[page*width+(x-x1)]=temp;//写入oled页缓冲区中当前页的当前列
        }
        page++;//下一页
    }
    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(lcd_panel_hdl,x1,y1_fullpage,x2+1,y2_fullpage+1,oled_buffer_partial));
    */
}

static void lvgl_init()
{
    /*
    1.lvgl初始化--创建显示对象,将显示对象与面板绑定
    2.分配绘图缓冲区--lvgl绘图后传入硬件缓冲区
    3.注册刷新回调函数--
    4.创建定时器--作为lvgl时钟
    */
    lv_init();
    display=lv_display_create(OLED_WIDTH,OLED_HEIGHT);//创建显示对象
    lv_display_set_user_data(display,lcd_panel_hdl);//绑定显示对象和面板
    //全局刷新模式
    size_t draw_buffer_size_fullscreen = OLED_WIDTH * OLED_HEIGHT / 8 + LVGL_PALETTE_SIZE;//lvgl*绘图*缓冲区,按照lvgl要求存储像素(像素数+8*8位调色盘)
    void *buffer=NULL;//lvgl绘图缓冲区指针
    buffer=heap_caps_calloc(1,draw_buffer_size_fullscreen,MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);//分配缓冲区内存
    assert(buffer);//检查内存分配是否成功
    lv_display_set_color_format(display,LV_COLOR_FORMAT_I1);//设置颜色格式,T1模式:显示色深1bit,前8字节为调色盘
    lv_display_set_buffers(display,buffer,NULL,draw_buffer_size_fullscreen,LV_DISPLAY_RENDER_MODE_FULL);//初始化绘图缓冲区
    /*局部刷新模式
    size_t draw_buffer_size_partial = OLED_WIDTH * OLED_HEIGHT / 8 /10 + LVGL_PALETTE_SIZE;//lvgl*绘图*缓冲区,按照lvgl要求存储像素(像素数+8*8位调色盘)
    void *buffer=NULL;//lvgl绘图缓冲区指针
    buffer=heap_caps_calloc(1,draw_buffer_size_partial,MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);//分配缓冲区内存
    assert(buffer);//检查内存分配是否成功
    lv_display_set_color_format(display,LV_COLOR_FORMAT_I1);//设置颜色格式,T1模式:显示色深1bit,前8字节为调色盘
    lv_display_set_buffers(display,buffer,NULL,draw_buffer_size_partial,LV_DISPLAY_RENDER_MODE_PARTIAL);//初始化绘图缓冲区
    */    
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

static void lvgl_task()
{
    uint32_t lvgl_delay_ms=0;
    while (1)
    {
        _lock_acquire(&lvgl_api_lock);//保持互斥锁,保护lvgl线程安全
        lvgl_delay_ms=lv_timer_handler();
        _lock_release(&lvgl_api_lock);//释放互斥锁
        lvgl_delay_ms=MAX(lvgl_delay_ms,LVGL_TASK_DELAY_MS_MIN);//取最小延时防止看门狗饿死
        lvgl_delay_ms=MIN(lvgl_delay_ms,LVGL_TASK_DELAY_MS_MAX);//取最大延时防止响应过慢
        usleep(1000*lvgl_delay_ms);//延时1000*lvgl_delay_ms微秒,即lvgl_delay_ms毫秒
    }
}

static void lvgl_ui(lv_display_t *disp)
{
    lv_obj_t *screen=lv_display_get_screen_active(disp);//获取当前屏幕对象
    lv_obj_t *label=lv_label_create(screen);//在屏幕上创建一个标签
    lv_label_set_long_mode(label,LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);//设置文本过长时显示模式为滚动循环
    lv_label_set_text(label,"Hello World.Hello LMD");//设置文本
    lv_obj_set_width(label,lv_display_get_horizontal_resolution(disp));//设置标签宽度为屏幕宽度
    lv_obj_align(label,LV_ALIGN_TOP_MID,0,0);//标签对齐到屏幕顶部中央
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
    xTaskCreatePinnedToCore(lvgl_task,"LVGL",4096,NULL,5,NULL,0);//创建lvgl任务
    _lock_acquire(&lvgl_api_lock);//保持互斥锁
    lvgl_ui(display);//显示
    _lock_release(&lvgl_api_lock);//释放互斥锁
}
