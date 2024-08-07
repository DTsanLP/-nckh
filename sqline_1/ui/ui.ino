#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <XPT2046_Touchscreen.h>

#include <esp_now.h>
#include <WiFi.h>
/*Don't forget to set Sketchbook location in File/Preferences to the path of your UI project (the parent foder of this INO file)*/
// Touch Screen pins
// ----------------------------

// The CYD touch uses some non-default
// SPI pins
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

SPIClass mySpi = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
uint16_t touchScreenMinimumX = 200, touchScreenMaximumX = 3700, touchScreenMinimumY = 240, touchScreenMaximumY = 3800;

/*Change to your screen resolution*/
static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];

// MESSAGE
typedef struct struct_message
{
    // char a[32];
    int seq;
    int mq_data;
    bool fl_data;
    float temp_data;
    float humi_data;
} struct_message;
struct_message myData;

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char *buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();

    lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
    if (ts.touched())
    {
        TS_Point p = ts.getPoint();
        // Some very basic auto calibration so it doesn't go out of range
        if (p.x < touchScreenMinimumX)
            touchScreenMinimumX = p.x;
        if (p.x > touchScreenMaximumX)
            touchScreenMaximumX = p.x;
        if (p.y < touchScreenMinimumY)
            touchScreenMinimumY = p.y;
        if (p.y > touchScreenMaximumY)
            touchScreenMaximumY = p.y;
        // Map this to the pixel position
        data->point.x = map(p.x, touchScreenMinimumX, touchScreenMaximumX, 1, screenWidth);  /* Touchscreen X calibration */
        data->point.y = map(p.y, touchScreenMinimumY, touchScreenMaximumY, 1, screenHeight); /* Touchscreen Y calibration */
        data->state = LV_INDEV_STATE_PR;

        // Serial.print("Touch x ");
        // Serial.print(data->point.x);
        // Serial.print(" y ");
        // Serial.println(data->point.y);
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&myData, incomingData, sizeof(myData));
    Serial.print("Bytes received: ");
    Serial.println(len);
    Serial.print("Info: ");
    Serial.println(myData.seq);
    Serial.print("MQ: ");
    Serial.println(myData.mq_data);
    Serial.print("Flame: ");
    Serial.println(myData.fl_data);
    Serial.print("Temperature: ");
    Serial.println(myData.temp_data);
    Serial.print("Humidity: ");
    Serial.println(myData.humi_data);
    Serial.println("-------");
}
void setup()
{
    Serial.begin(115200); /* prepare for possible serial debug */

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK)
    {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println(LVGL_Arduino);
    Serial.println("I am LVGL_Arduino");

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

    mySpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    ts.begin(mySpi);
    ts.setRotation(3);

    tft.begin();        /* TFT init */
    tft.setRotation(3); /* Landscape orientation, flipped */
    tft.invertDisplay(1);
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    ui_init();

    Serial.println("Setup done");
}

void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    reload_Value();
    delay(5);
}

void reload_Value()
{
    lv_obj_set_style_bg_color(ui_Btn101, myData.fl_data == 0 || myData.mq_data >= 500 /*|| myData.temp_data >= 40 */ ? lv_color_hex(0x210000) : lv_color_hex(0x217C69), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_color_t current_bg_color = lv_obj_get_style_bg_color(ui_Btn101, LV_PART_MAIN | LV_STATE_DEFAULT);

    if (current_bg_color.full == lv_color_hex(0x210000).full)
    {
        lv_obj_clear_flag(ui_LbSOS, LV_OBJ_FLAG_HIDDEN);
        // lv_label_set_text(ui_LbSOS, "SOS ROOM:");
    }
    else
    {
        lv_obj_add_flag(ui_LbSOS, LV_OBJ_FLAG_HIDDEN);
    }

    /// Show
    char buffer[5];
    sprintf(buffer, "%04d", myData.mq_data);
    lv_label_set_text(ui_ValSmoke, buffer);
    lv_label_set_text(ui_ValFire, myData.fl_data == 0 ? "YES" : "NO");
    lv_arc_set_value(ui_ArcTemp, myData.temp_data);
    lv_arc_set_value(ui_ArcHumi, myData.humi_data);

    char humi_data_str[6];
    dtostrf(myData.humi_data, 3, 1, humi_data_str);
    lv_label_set_text(ui_ValHumi, humi_data_str);
    char temp_data_str[6];
    dtostrf(myData.temp_data, 4, 1, temp_data_str);
    lv_label_set_text(ui_ValTemp, temp_data_str);

    /// Conditions
    if (myData.mq_data >= 500)
    {
        lv_obj_clear_flag(ui_ImgSmoke, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(ui_ImgSmoke, LV_OBJ_FLAG_HIDDEN);
    }

    if (myData.fl_data == 0 /*|| myData.temp_data >= 40*/)
    {
        lv_obj_clear_flag(ui_ImgFire, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        lv_obj_add_flag(ui_ImgFire, LV_OBJ_FLAG_HIDDEN);
    }
    // lv_obj_clear_flag(ui_BtnSafe, LV_OBJ_FLAG_HIDDEN);

    delay(100);
}
