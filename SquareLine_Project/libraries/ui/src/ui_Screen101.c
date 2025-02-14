// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "ui.h"

void ui_Screen101_screen_init(void)
{
    ui_Screen101 = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Screen101, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Screen101, lv_color_hex(0xBDDEEE), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Screen101, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel2 = lv_obj_create(ui_Screen101);
    lv_obj_set_width(ui_Panel2, 64);
    lv_obj_set_height(ui_Panel2, 32);
    lv_obj_set_x(ui_Panel2, 0);
    lv_obj_set_y(ui_Panel2, -100);
    lv_obj_set_align(ui_Panel2, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_LabelScreen101 = lv_label_create(ui_Screen101);
    lv_obj_set_width(ui_LabelScreen101, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelScreen101, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LabelScreen101, 0);
    lv_obj_set_y(ui_LabelScreen101, -100);
    lv_obj_set_align(ui_LabelScreen101, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelScreen101, "101");
    lv_obj_set_style_text_color(ui_LabelScreen101, lv_color_hex(0x090101), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LabelScreen101, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LabelScreen101, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_FlameSensor = lv_obj_create(ui_Screen101);
    lv_obj_remove_style_all(ui_FlameSensor);
    lv_obj_set_width(ui_FlameSensor, 100);
    lv_obj_set_height(ui_FlameSensor, 100);
    lv_obj_set_x(ui_FlameSensor, -75);
    lv_obj_set_y(ui_FlameSensor, -20);
    lv_obj_set_align(ui_FlameSensor, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_FlameSensor, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_FlameSensor, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_FlameSensor, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_FlameSensor, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_FlameSensor, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_FlameSensor, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_FlameSensor, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelFL101 = lv_label_create(ui_FlameSensor);
    lv_obj_set_width(ui_LabelFL101, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelFL101, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_LabelFL101, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_LabelFL101, "Flame");

    ui_LabelValueFL101 = lv_label_create(ui_FlameSensor);
    lv_obj_set_width(ui_LabelValueFL101, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelValueFL101, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_LabelValueFL101, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelValueFL101, "Value");

    ui_FireImg101 = lv_img_create(ui_FlameSensor);
    lv_img_set_src(ui_FireImg101, &ui_img_fire_border_resize_png);
    lv_obj_set_width(ui_FireImg101, 100);
    lv_obj_set_height(ui_FireImg101, 100);
    lv_obj_set_x(ui_FireImg101, 0);
    lv_obj_set_y(ui_FireImg101, 100);
    lv_obj_set_align(ui_FireImg101, LV_ALIGN_BOTTOM_MID);
    lv_obj_add_flag(ui_FireImg101, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_FireImg101, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_zoom(ui_FireImg101, 180);

    ui_SmokeSensor = lv_obj_create(ui_Screen101);
    lv_obj_remove_style_all(ui_SmokeSensor);
    lv_obj_set_width(ui_SmokeSensor, 100);
    lv_obj_set_height(ui_SmokeSensor, 103);
    lv_obj_set_x(ui_SmokeSensor, 75);
    lv_obj_set_y(ui_SmokeSensor, -20);
    lv_obj_set_align(ui_SmokeSensor, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_SmokeSensor, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_SmokeSensor, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_SmokeSensor, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_SmokeSensor, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_SmokeSensor, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_SmokeSensor, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_SmokeSensor, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LabelMQ101 = lv_label_create(ui_SmokeSensor);
    lv_obj_set_width(ui_LabelMQ101, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelMQ101, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_LabelMQ101, LV_ALIGN_TOP_MID);
    lv_label_set_text(ui_LabelMQ101, "Smoke");

    ui_LabelValueMQ101 = lv_label_create(ui_SmokeSensor);
    lv_obj_set_width(ui_LabelValueMQ101, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LabelValueMQ101, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_LabelValueMQ101, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LabelValueMQ101, "Value");

    ui_SmokeImg101 = lv_img_create(ui_SmokeSensor);
    lv_img_set_src(ui_SmokeImg101, &ui_img_smoke_border_resize_png);
    lv_obj_set_width(ui_SmokeImg101, LV_SIZE_CONTENT);   /// 100
    lv_obj_set_height(ui_SmokeImg101, LV_SIZE_CONTENT);    /// 100
    lv_obj_set_x(ui_SmokeImg101, 0);
    lv_obj_set_y(ui_SmokeImg101, 100);
    lv_obj_set_align(ui_SmokeImg101, LV_ALIGN_BOTTOM_MID);
    lv_obj_add_flag(ui_SmokeImg101, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_SmokeImg101, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_zoom(ui_SmokeImg101, 180);

    ui_Switch101 = lv_switch_create(ui_Screen101);
    lv_obj_set_width(ui_Switch101, 60);
    lv_obj_set_height(ui_Switch101, 30);
    lv_obj_set_x(ui_Switch101, -6);
    lv_obj_set_y(ui_Switch101, 65);
    lv_obj_set_align(ui_Switch101, LV_ALIGN_CENTER);

    lv_obj_set_style_bg_color(ui_Switch101, lv_color_hex(0xF96429), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Switch101, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_Switch101, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Switch101, 255, LV_PART_KNOB | LV_STATE_DEFAULT);

    ui_ButtonBack101 = lv_btn_create(ui_Screen101);
    lv_obj_set_width(ui_ButtonBack101, 40);
    lv_obj_set_height(ui_ButtonBack101, 30);
    lv_obj_set_x(ui_ButtonBack101, -130);
    lv_obj_set_y(ui_ButtonBack101, -100);
    lv_obj_set_align(ui_ButtonBack101, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_ButtonBack101, lv_color_hex(0x278B6A), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ButtonBack101, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_ImgBack101 = lv_img_create(ui_ButtonBack101);
    lv_img_set_src(ui_ImgBack101, &ui_img_back_png);
    lv_obj_set_width(ui_ImgBack101, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_ImgBack101, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_ImgBack101, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_ImgBack101, LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_ImgBack101, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_img_set_zoom(ui_ImgBack101, 10);

    lv_obj_add_event_cb(ui_FireImg101, ui_event_FireImg101, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_SmokeImg101, ui_event_SmokeImg101, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ButtonBack101, ui_event_ButtonBack101, LV_EVENT_ALL, NULL);

}
