#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <XPT2046_Touchscreen.h>

// DC MAC D1 R32: {0x08, 0xD1, 0xF9, 0x35, 0x1F, 0xB0};
#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#define BUZ_PIN 27
#define BAT_PIN 22
#define MAX_PEERS 10
uint8_t broadcastAddress[] = {0x08, 0xD1, 0xF9, 0x35, 0x1F, 0xB0};
// uint8_t broadcastAddress[6];
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
  uint8_t add[6];
  unsigned int seq;
  int mq_data;
  bool fl_data;
  float temp_data;
  uint8_t humi_data;
} struct_message;
static struct_message myData;
static std::vector<struct_message> boardsStruct;
// struct_message boardsStruct[48];
typedef struct control_message
{
  unsigned int seq;
  bool buz_ctrl;
  bool dcm_ctrl;
} control_message;
static control_message ctrlData;
static uint8_t checkadd[6];
esp_now_peer_info_t peerInfo;

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

  bool boardFound = false;
  for (auto &board : boardsStruct)
  {
    if (board.seq == myData.seq)
    {
      for (int i = 0; i < 6; i++)
      {
        board.add[i] = myData.add[i];
      }
      board.temp_data = myData.temp_data;
      board.humi_data = myData.humi_data;
      board.mq_data = myData.mq_data;
      board.fl_data = myData.fl_data;
      memcpy(peerInfo.peer_addr, board.add, 6);
      if (esp_now_add_peer(&peerInfo) != ESP_OK)
      {
        Serial.println("Failed to add peer");
        return;
      }
      boardFound = true;
      break;
    }
  }

  // Nếu không tìm thấy board trong vector, thêm mới
  if (!boardFound)
  {
    boardsStruct.push_back(myData);
  }
  // Serial.println(myData.add);
  // Serial.println(myData.temp_data);
  // Serial.println(myData.humi_data);
  // Serial.println(myData.mq_data);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup()
{
  Serial.begin(115200); /* prepare for possible serial debug */
  pinMode(BUZ_PIN, OUTPUT);
  pinMode(BAT_PIN, OUTPUT);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_now_register_send_cb(OnDataSent);

  // memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

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
  digitalWrite(BUZ_PIN, HIGH);
}

void loop()
{
  lv_timer_handler(); /* let the GUI do its work */
  reload_Value();
  delay(5);
}

void reload_Value()
{
  for (auto &board : boardsStruct)
  {
    if (board.seq == 101)
    {
      lv_obj_set_style_bg_color(ui_Btn101, board.fl_data == 0 || board.mq_data >= 500 || board.temp_data >= 45 ? lv_color_hex(0x210000) : lv_color_hex(0x217C69), LV_PART_MAIN | LV_STATE_DEFAULT);
      lv_color_t current_bg_color = lv_obj_get_style_bg_color(ui_Btn101, LV_PART_MAIN | LV_STATE_DEFAULT);
      if (current_bg_color.full == lv_color_hex(0x210000).full)
      {
        lv_obj_clear_flag(ui_LbSOS, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_BtnSafe, LV_OBJ_FLAG_HIDDEN);
        char *text = lv_label_get_text(ui_Lb101);
        lv_label_set_text(ui_LbSOS, text);
        digitalWrite(BUZ_PIN, LOW);
        if (board.fl_data == 0)
        {
          lv_obj_clear_flag(ui_Battery, LV_OBJ_FLAG_HIDDEN);
        }
      }
      else
      {
        // lv_obj_add_flag(ui_LbSOS, LV_OBJ_FLAG_HIDDEN);
        // digitalWrite(BUZ_PIN, HIGH);
      }
    }
    else
    {
      for (int i = 0; i < roomButtonCount; i++)
      {
        if (board.seq == roomButtons[i].id)
        {
          lv_obj_set_style_bg_color(roomButtons[i].btn, board.fl_data == 0 || board.mq_data >= 500 || board.temp_data >= 45 ? lv_color_hex(0x210000) : lv_color_hex(0x217C69), LV_PART_MAIN | LV_STATE_DEFAULT);
          lv_color_t current_bg_color = lv_obj_get_style_bg_color(roomButtons[i].btn, LV_PART_MAIN | LV_STATE_DEFAULT);
          if (current_bg_color.full == lv_color_hex(0x210000).full)
          {
            lv_obj_clear_flag(ui_LbSOS, LV_OBJ_FLAG_HIDDEN);
            lv_obj_clear_flag(ui_BtnSafe, LV_OBJ_FLAG_HIDDEN);
            char *text = lv_label_get_text(roomButtons[i].label);
            lv_label_set_text(ui_LbSOS, text);
            digitalWrite(BUZ_PIN, LOW);
            if (board.fl_data == 0)
            {
              lv_obj_clear_flag(ui_Battery, LV_OBJ_FLAG_HIDDEN);
            }
          }
          else
          {
            // lv_obj_add_flag(ui_LbSOS, LV_OBJ_FLAG_HIDDEN);
          }
        }
      }
    }
    Serial.println(clicked);
    Serial.println(lv_label_get_text(ui_LbRoom));
    Serial.println("------");
  }
  /// Show
  if (clicked == true)
  {
    char *text_room = lv_label_get_text(ui_LbRoom);
    unsigned int check_room = atoi(text_room);
    for (auto &board : boardsStruct)
    {
      if (check_room == board.seq)
      {
        for (int i = 0; i < 6; i++)
        {
          checkadd[i] = board.add[i];
        }
        lv_label_set_text_fmt(ui_ValSmoke, "%d", board.mq_data);
        lv_label_set_text(ui_ValFire, board.fl_data == 0 ? "YES" : "NO");

        lv_label_set_text_fmt(ui_ValHumi, "%d", board.humi_data);

        char temp_data_str[6];
        dtostrf(board.temp_data, 4, 1, temp_data_str);
        lv_label_set_text(ui_ValTemp, temp_data_str);

        lv_arc_set_value(ui_ArcTemp, board.temp_data);
        lv_arc_set_value(ui_ArcHumi, board.humi_data);

        /// Conditions
        if (board.mq_data >= 500)
        {
          lv_obj_clear_flag(ui_ImgSmoke, LV_OBJ_FLAG_HIDDEN);
          lv_obj_clear_flag(ui_BtnSafe, LV_OBJ_FLAG_HIDDEN);
          lv_label_set_text(ui_LbSafe, text_room);
        }
        else
        {
          lv_obj_add_flag(ui_ImgSmoke, LV_OBJ_FLAG_HIDDEN);
        }

        if (board.fl_data == 0 || board.temp_data >= 45)
        {
          lv_obj_clear_flag(ui_ImgFire, LV_OBJ_FLAG_HIDDEN);
          lv_obj_clear_flag(ui_BtnSafe, LV_OBJ_FLAG_HIDDEN);
          lv_label_set_text(ui_LbSafe, text_room);
        }
        else
        {
          lv_obj_add_flag(ui_ImgFire, LV_OBJ_FLAG_HIDDEN);
        }
        break;
      }
    }
    if (Batclicked == true)
    {
      ctrlData.seq = check_room;
      ctrlData.dcm_ctrl = true;
      digitalWrite(BAT_PIN, HIGH);
    }
    
    if (SOSclicked == true)
    {
      ctrlData.seq = check_room;
      ctrlData.buz_ctrl = false;
      ctrlData.dcm_ctrl = false;
      digitalWrite(BUZ_PIN, HIGH);
                  // lv_obj_set_style_bg_color(roomButtons[i].btn, lv_color_hex(0x210000), LV_PART_MAIN|LV_STATE_DEFAULT);
      digitalWrite(BAT_PIN, LOW);

    }
    else
    {
      ctrlData.seq = check_room;
      ctrlData.buz_ctrl = true;

    }
  }
  Serial.println(ctrlData.buz_ctrl);
  Serial.println("=========");
  // Serial.println(checkadd);
  // memcpy(broadcastAddress, checkadd, 6);
  esp_err_t result = esp_now_send(checkadd, (uint8_t *)&ctrlData, sizeof(ctrlData));
  if (result == ESP_OK)
  {
    Serial.println(ctrlData.buz_ctrl);
    if (ctrlData.buz_ctrl == false)
    {
      lv_obj_add_flag(ui_BtnSafe, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_LbSOS, LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(ui_Battery, LV_OBJ_FLAG_HIDDEN);
      SOSclicked = false;
      Batclicked = false;
      delay(200);
    }
    Serial.println("Sent with success");
  }
  else
  {
    Serial.println("Error sending the data");
  }

  delay(50);
}