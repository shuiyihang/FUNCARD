#include <stdio.h>

/* freertos includes */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_freertos_hooks.h"
#include "lvgl_gui.h"
#include "esp_log.h"

static const char *TAG = "example_lvgl";
/**
 * 禁用触摸屏
 * 交换颜色
*/

#define EC11_A_PIN  3
#define EC11_B_PIN  9
#define EC11_E_PIN  10

static xQueueHandle gpio_evt_queue = NULL;

static int32_t encoder_diff;
static lv_indev_state_t encoder_state;

static lv_indev_drv_t indev_drv;

static scr_interface_driver_t *get_8080_iface(void)
{
    i2s_lcd_config_t i2s_lcd_cfg = {
        .data_width  = 8,
        .pin_data_num = {
#ifdef CONFIG_IDF_TARGET_ESP32
            // 19, 21, 0, 22, 23, 33, 32, 27, 25, 26, 12, 13, 14, 15, 2, 4,
            40,39,38,37,36,35,34,33,
#else
            40,39,38,37,36,35,34,33,
#endif
        },
        .pin_num_cs = 4,
#ifdef CONFIG_IDF_TARGET_ESP32
        .pin_num_wr = 6,
        .pin_num_rs = 5,
#else
        .pin_num_wr = 6,//write enable
        .pin_num_rs = 5,//data/cmd select
#endif
        .clk_freq = 20000000,
        .i2s_port = I2S_NUM_0,
        .buffer_size = 32000,
        .swap_data = true,
    };

    scr_interface_driver_t *iface_drv_i2s;
    scr_interface_create(SCREEN_IFACE_8080, &i2s_lcd_cfg, &iface_drv_i2s);
    return iface_drv_i2s;
}


static void encoder_isr_handler(void* arg)
{
    uint32_t status = 0;
    uint32_t gpio_num = (uint32_t) arg;
    if(!gpio_get_level(EC11_B_PIN)){
        //逆时针旋转
        if(!gpio_get_level(gpio_num)) {//fix esp32 
            encoder_diff--;
            encoder_state = LV_INDEV_STATE_REL;

            status = 1;
            xQueueSendFromISR(gpio_evt_queue, &status, NULL);
        }
    }else{
        //顺时针旋转
        if(!gpio_get_level(gpio_num)){
            encoder_diff++;
            encoder_state = LV_INDEV_STATE_REL;

            xQueueSendFromISR(gpio_evt_queue, &status, NULL);
        } 
            
    }
}

static void key_isr_handler(void *arg)//边沿进入可能有问题
{
    uint32_t status = 2;
    encoder_state = LV_INDEV_STATE_PR;
    xQueueSendFromISR(gpio_evt_queue, &status, NULL);
}

void gpio_task_example(void* arg)
{
    uint32_t status;
    uint32_t hold_cnt = 0;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &status, 0)) {
            if(status == 1){
                hold_cnt = 0;
                ESP_LOGI(TAG, "encoder Clockwise \n");
            }else if(status == 0){
                hold_cnt = 0;
                ESP_LOGI(TAG, "encoder Counterclockwise \n");
            }else{
                ESP_LOGI(TAG, "encoder pressed \n");
                // encoder_state = LV_INDEV_STATE_REL;
            }
        }
        if(status == 2){
            if(!gpio_get_level(EC11_E_PIN)){
                encoder_state = LV_INDEV_STATE_PR;
                hold_cnt++;
                ESP_LOGI(TAG, "encoder pressed:%d \n",hold_cnt);
            }else{
                encoder_state = LV_INDEV_STATE_REL;
                hold_cnt = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

bool encoder_read(lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    data->enc_diff = encoder_diff;
    data->state = encoder_state;
    encoder_diff = 0;
    return false;
}


/**
 * lvgl的log为啥没有
 * 
 * 滚动条必须要选中吗
*/
void app_main()
{
    /**
     * tft lcd init
    */
    scr_driver_t lcd;
    scr_interface_driver_t *iface_drv = get_8080_iface();
    scr_controller_config_t lcd_cfg = {0};
    lcd_cfg.interface_drv = iface_drv;
    lcd_cfg.pin_num_rst = 2;
#ifdef CONFIG_IDF_TARGET_ESP32
    lcd_cfg.pin_num_bckl = 1;
#else
    lcd_cfg.pin_num_bckl = 1;
#endif
    lcd_cfg.rst_active_level = 0;
    lcd_cfg.bckl_active_level = 1;
    lcd_cfg.offset_hor = 0;
    lcd_cfg.offset_ver = 0;
    lcd_cfg.width = 240;
    lcd_cfg.height = 320;
    lcd_cfg.rotate = SCR_DIR_BTLR;//SCR_DIR_LRBT

    scr_find_driver(SCREEN_CONTROLLER_ST7789, &lcd);
    lcd.init(&lcd_cfg);//init


    // lcd_color_test(&lcd);

    /**
     * encoder init
    */
    gpio_config_t EC11_A_IO;

    EC11_A_IO.intr_type = GPIO_INTR_NEGEDGE;//上升沿中断
    EC11_A_IO.mode = GPIO_MODE_INPUT;
    EC11_A_IO.pull_up_en = 1;
    EC11_A_IO.pull_down_en = 0;

    EC11_A_IO.pin_bit_mask = (1ULL << EC11_A_PIN) | (1ULL << EC11_E_PIN);;

    gpio_config(&EC11_A_IO);

    //install gpio isr service
    gpio_install_isr_service(0);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(EC11_A_PIN, encoder_isr_handler, (void*) EC11_A_PIN);
    gpio_isr_handler_add(EC11_E_PIN, key_isr_handler, (void*) EC11_E_PIN);
////////////////////普通io读操作
    gpio_config_t EC11_B_IO;

    EC11_B_IO.intr_type = GPIO_INTR_DISABLE;//上升沿中断
    EC11_B_IO.mode = GPIO_MODE_INPUT;
    EC11_B_IO.pull_up_en = 1;
    EC11_B_IO.pull_down_en = 0;

    EC11_B_IO.pin_bit_mask = (1ULL << EC11_B_PIN);


    gpio_config(&EC11_B_IO);
    

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);


    /* Initialize LittlevGL GUI */
    lvgl_init(&lcd, NULL);

    lv_obj_t *roller1 = lv_roller_create(lv_scr_act(),NULL);
    lv_roller_set_options(roller1,
                        "January\n"
                        "February\n"
                        "March\n"
                        "April\n"
                        "May\n"
                        "June\n"
                        "July\n"
                        "August\n"
                        "September\n"
                        "October\n"
                        "November\n"
                        "December",
                        LV_ROLLER_MODE_INFINITE);

    lv_roller_set_visible_row_count(roller1, 4);

    lv_obj_align(roller1, lv_scr_act(), LV_ALIGN_IN_TOP_MID, 0, 0);
    lv_roller_set_selected(roller1, 2, LV_ANIM_OFF);


    lv_group_t *g = lv_group_create();

    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = encoder_read;
    lv_indev_t *enc_indev = lv_indev_drv_register(&indev_drv);

    lv_group_add_obj(g,roller1);

    lv_indev_set_group(enc_indev, g);
    

    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
}
