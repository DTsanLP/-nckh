// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#ifndef _SQUARELINE_PROJECT_UI_H
#define _SQUARELINE_PROJECT_UI_H

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

#include "ui_helpers.h"
#include "ui_comp.h"
#include "ui_comp_hook.h"
#include "ui_events.h"

void FloatUp_Animation(lv_obj_t * TargetObject, int delay);
void FloatDown_Animation(lv_obj_t * TargetObject, int delay);
// SCREEN: ui_MainScreen
void ui_MainScreen_screen_init(void);
extern lv_obj_t * ui_MainScreen;
void ui_event_Button101(lv_event_t * e);
extern lv_obj_t * ui_Button101;
extern lv_obj_t * ui_Label101;
void ui_event_Button102(lv_event_t * e);
extern lv_obj_t * ui_Button102;
extern lv_obj_t * ui_Label102;
extern lv_obj_t * ui_Button103;
extern lv_obj_t * ui_Label103;
extern lv_obj_t * ui_Button104;
extern lv_obj_t * ui_Label104;
void ui_event_Button201(lv_event_t * e);
extern lv_obj_t * ui_Button201;
extern lv_obj_t * ui_Label201;
extern lv_obj_t * ui_Button202;
extern lv_obj_t * ui_Label202;
extern lv_obj_t * ui_Button203;
extern lv_obj_t * ui_Label203;
extern lv_obj_t * ui_Button8;
extern lv_obj_t * ui_Label8;
extern lv_obj_t * ui_Button9;
extern lv_obj_t * ui_Label9;
extern lv_obj_t * ui_Button10;
extern lv_obj_t * ui_Label10;
extern lv_obj_t * ui_Button11;
extern lv_obj_t * ui_Label11;
extern lv_obj_t * ui_Button12;
extern lv_obj_t * ui_Label12;
extern lv_obj_t * ui_Label13;
extern lv_obj_t * ui_Label14;
extern lv_obj_t * ui_Panel1;
// SCREEN: ui_Screen101
void ui_Screen101_screen_init(void);
extern lv_obj_t * ui_Screen101;
extern lv_obj_t * ui_Panel2;
extern lv_obj_t * ui_LabelScreen101;
extern lv_obj_t * ui_FlameSensor;
extern lv_obj_t * ui_LabelFL101;
extern lv_obj_t * ui_LabelValueFL101;
void ui_event_FireImg101(lv_event_t * e);
extern lv_obj_t * ui_FireImg101;
extern lv_obj_t * ui_SmokeSensor;
extern lv_obj_t * ui_LabelMQ101;
extern lv_obj_t * ui_LabelValueMQ101;
void ui_event_SmokeImg101(lv_event_t * e);
extern lv_obj_t * ui_SmokeImg101;
extern lv_obj_t * ui_Switch101;
void ui_event_ButtonBack101(lv_event_t * e);
extern lv_obj_t * ui_ButtonBack101;
extern lv_obj_t * ui_ImgBack101;
// SCREEN: ui_Screen102
void ui_Screen102_screen_init(void);
extern lv_obj_t * ui_Screen102;
extern lv_obj_t * ui_Panel4;
extern lv_obj_t * ui_LabelScreen102;
extern lv_obj_t * ui_FlameSensor102;
extern lv_obj_t * ui_LabelFL102;
extern lv_obj_t * ui_LabelValueFL102;
void ui_event_FireImg102(lv_event_t * e);
extern lv_obj_t * ui_FireImg102;
extern lv_obj_t * ui_SmokeSensor102;
extern lv_obj_t * ui_LabelMQ102;
extern lv_obj_t * ui_LabelValueMQ102;
void ui_event_SmokeImg102(lv_event_t * e);
extern lv_obj_t * ui_SmokeImg102;
extern lv_obj_t * ui_Switch102;
void ui_event_ButtonBack102(lv_event_t * e);
extern lv_obj_t * ui_ButtonBack102;
extern lv_obj_t * ui_ImgBack102;
// SCREEN: ui_Screen201
void ui_Screen201_screen_init(void);
extern lv_obj_t * ui_Screen201;
extern lv_obj_t * ui_Panel5;
extern lv_obj_t * ui_LabelScreen201;
extern lv_obj_t * ui_FlameSensor201;
extern lv_obj_t * ui_LabelFL201;
extern lv_obj_t * ui_LabelValueFL201;
void ui_event_FireImg201(lv_event_t * e);
extern lv_obj_t * ui_FireImg201;
extern lv_obj_t * ui_SmokeSensor201;
extern lv_obj_t * ui_LabelMQ201;
extern lv_obj_t * ui_LabelValueMQ201;
void ui_event_SmokeImg201(lv_event_t * e);
extern lv_obj_t * ui_SmokeImg201;
extern lv_obj_t * ui_Switch201;
void ui_event_ButtonBack201(lv_event_t * e);
extern lv_obj_t * ui_ButtonBack201;
extern lv_obj_t * ui_ImgBack201;
extern lv_obj_t * ui____initial_actions0;


LV_IMG_DECLARE(ui_img_fire_border_resize_png);    // assets/fire_border_resize.png
LV_IMG_DECLARE(ui_img_smoke_border_resize_png);    // assets/smoke_border_resize.png
LV_IMG_DECLARE(ui_img_back_png);    // assets/back.png






void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
