// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.6
// Project name: sqline_1

#ifndef _SQLINE_1_UI_H
#define _SQLINE_1_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif

#include "RoomButton.h"
extern RoomButton *roomButtons;


#include "ui_helpers.h"
#include "ui_events.h"

// SCREEN: ui_Home
void ui_Home_screen_init(void);
extern lv_obj_t * ui_Home;
extern lv_obj_t * ui_LbName;
extern lv_obj_t * ui_GroupBtn;
extern lv_obj_t * ui_GroupScroll;
void button_event_handler(lv_event_t * e);
void create_room_button(lv_obj_t * parent, int room_number, int x, int y);
extern int roomButtonCount ;
void add_room_button(lv_obj_t *btn, int id, lv_obj_t*label);
extern lv_obj_t * ui_Btn101;
extern lv_obj_t * ui_Lb101;
extern bool clicked;
extern bool SOSclicked;
extern bool Batclicked;
extern lv_obj_t * ui_PanelTemp;
void ui_event_BtnAdd(lv_event_t * e);
extern lv_obj_t * ui_BtnAdd;
extern lv_obj_t * ui_LbAdd;
extern lv_obj_t * ui_TextAreaHome;
void ui_event_KeyboardHome(lv_event_t * e);
// void show_warning_message();
extern lv_obj_t * ui_KeyboardHome;
extern lv_obj_t * ui_LbSOS;
// SCREEN: ui_Room
void ui_Room_screen_init(void);
extern lv_obj_t * ui_Room;
extern lv_obj_t * ui_LbRoom;
void ui_event_BtnBack(lv_event_t * e);
extern lv_obj_t * ui_BtnBack;
extern lv_obj_t * ui_ImgBack;
extern lv_obj_t * ui_ContainHumiTemp;
void ui_event_ArcHumi(lv_event_t * e);
extern lv_obj_t * ui_ArcHumi;
extern lv_obj_t * ui_ValHumi;
extern lv_obj_t * ui_LbHumi;
void ui_event_ArcTemp(lv_event_t * e);
extern lv_obj_t * ui_ArcTemp;
extern lv_obj_t * ui_ValTemp;
extern lv_obj_t * ui_LbTemp;
extern lv_obj_t * ui_ContainFire;
extern lv_obj_t * ui_ValSmoke;
extern lv_obj_t * ui_ValFire;
extern lv_obj_t * ui_ImgFire;
extern lv_obj_t * ui_ImgSmoke;
void ui_event_BtnSafe(lv_event_t * e);
extern lv_obj_t * ui_BtnSafe;
extern lv_obj_t * ui_LbSafe;
extern lv_obj_t * ui_Battery;
extern lv_obj_t * ui_LbBat;
void ui_event_Battery(lv_event_t *e);
extern lv_obj_t * ui____initial_actions0;


LV_IMG_DECLARE(ui_img_back_png);    // assets/back.png
LV_IMG_DECLARE(ui_img_fire_border_resize_png);    // assets/fire_border_resize.png
LV_IMG_DECLARE(ui_img_smoke_border_resize_png);    // assets/smoke_border_resize.png






void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
