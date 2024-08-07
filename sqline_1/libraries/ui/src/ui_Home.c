// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.1
// LVGL version: 8.3.6
// Project name: sqline_1

#include "ui.h"

void ui_Home_screen_init(void)
{
    ui_Home = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_Home, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Home, lv_color_hex(0xB8DFE9), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Home, 255, LV_PART_MAIN | LV_STATE_DEFAULT);


    ui_LbName = lv_label_create(ui_Home);
    lv_obj_set_width(ui_LbName, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LbName, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LbName, 0);
    lv_obj_set_y(ui_LbName, -105);
    lv_obj_set_align(ui_LbName, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LbName, "APARTMENT");
    lv_obj_set_style_text_color(ui_LbName, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LbName, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LbName, &lv_font_montserrat_16, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_GroupBtn = lv_obj_create(ui_Home);
    lv_obj_remove_style_all(ui_GroupBtn);
    lv_obj_set_width(ui_GroupBtn, 320);
    lv_obj_set_height(ui_GroupBtn, 170);
    lv_obj_set_align(ui_GroupBtn, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_GroupBtn, LV_OBJ_FLAG_CLICKABLE);      /// Flags
    lv_obj_set_scrollbar_mode(ui_GroupBtn, LV_SCROLLBAR_MODE_ACTIVE);

    lv_obj_set_style_bg_color(ui_GroupBtn, lv_color_hex(0x503838), LV_PART_SCROLLBAR | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_GroupBtn, 255, LV_PART_SCROLLBAR | LV_STATE_DEFAULT);

    ui_GroupScroll = lv_obj_create(ui_GroupBtn);
    lv_obj_remove_style_all(ui_GroupScroll);
    lv_obj_set_width(ui_GroupScroll, 640);
    lv_obj_set_height(ui_GroupScroll, 380);
    lv_obj_set_x(ui_GroupScroll, 160);
    lv_obj_set_y(ui_GroupScroll, 100);
    lv_obj_set_align(ui_GroupScroll, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_GroupScroll, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_scrollbar_mode(ui_GroupScroll, LV_SCROLLBAR_MODE_ON);

    ui_Btn101 = lv_btn_create(ui_GroupScroll);
    lv_obj_set_width(ui_Btn101, 60);
    lv_obj_set_height(ui_Btn101, 40);
    lv_obj_set_x(ui_Btn101, -280);
    lv_obj_set_y(ui_Btn101, -165);
    lv_obj_set_align(ui_Btn101, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_Btn101, lv_color_hex(0x1EA686), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Btn101, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(ui_Btn101, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_opa(ui_Btn101, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_border_width(ui_Btn101, 1, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Lb101 = lv_label_create(ui_Btn101);
    lv_obj_set_width(ui_Lb101, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Lb101, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Lb101, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Lb101, "101");
    lv_obj_add_event_cb(ui_Btn101, button_event_handler, LV_EVENT_CLICKED, NULL);

    // create_room_button(ui_GroupScroll, 101, -280, -165);
    create_room_button(ui_GroupScroll, 102, -200, -165);
    create_room_button(ui_GroupScroll, 103, -120, -165);
    create_room_button(ui_GroupScroll, 104, -40, -165);
    create_room_button(ui_GroupScroll, 105, 40, -165);
    create_room_button(ui_GroupScroll, 106, 120, -165);
    create_room_button(ui_GroupScroll, 107, 200, -165);   

    create_room_button(ui_GroupScroll, 201, -280, -105);   
    create_room_button(ui_GroupScroll, 202, -200, -105);
    create_room_button(ui_GroupScroll, 203, -120, -105);
    create_room_button(ui_GroupScroll, 204, -40, -105);
    create_room_button(ui_GroupScroll, 205, 40, -105);

    create_room_button(ui_GroupScroll, 301, -280, -45);
    create_room_button(ui_GroupScroll, 302, -200, -45);
    create_room_button(ui_GroupScroll, 303, -120, -45);
    create_room_button(ui_GroupScroll, 304, -40, -45);
    create_room_button(ui_GroupScroll, 305, 40, -45);

    create_room_button(ui_GroupScroll, 401, -280, 15);
    create_room_button(ui_GroupScroll, 402, -200, 15);
    create_room_button(ui_GroupScroll, 403, -120, 15);
    create_room_button(ui_GroupScroll, 404, -40, 15);
    create_room_button(ui_GroupScroll, 405, 40, 15);

    ui_PanelTemp = lv_obj_create(ui_Home);
    lv_obj_set_width(ui_PanelTemp, 320);
    lv_obj_set_height(ui_PanelTemp, 240);
    lv_obj_set_align(ui_PanelTemp, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_PanelTemp, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_clear_flag(ui_PanelTemp, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_BtnAdd = lv_btn_create(ui_Home);
    lv_obj_set_width(ui_BtnAdd, 40);
    lv_obj_set_height(ui_BtnAdd, 30);
    lv_obj_set_x(ui_BtnAdd, 135);
    lv_obj_set_y(ui_BtnAdd, 100);
    lv_obj_set_align(ui_BtnAdd, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_BtnAdd, lv_color_hex(0x9E9E9E), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_BtnAdd, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_LbAdd = lv_label_create(ui_BtnAdd);
    lv_obj_set_width(ui_LbAdd, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LbAdd, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_LbAdd, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LbAdd, "+");
    lv_obj_set_style_text_color(ui_LbAdd, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LbAdd, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_LbAdd, &lv_font_montserrat_20, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_TextAreaHome = lv_textarea_create(ui_Home);
    lv_obj_set_width(ui_TextAreaHome, 280);
    lv_obj_set_height(ui_TextAreaHome, LV_SIZE_CONTENT);    /// 70
    lv_obj_set_x(ui_TextAreaHome, 0);
    lv_obj_set_y(ui_TextAreaHome, -80);
    lv_obj_set_align(ui_TextAreaHome, LV_ALIGN_CENTER);
    lv_textarea_set_max_length(ui_TextAreaHome, 3);
    lv_textarea_set_placeholder_text(ui_TextAreaHome, "Enter Number: ");
    lv_textarea_set_one_line(ui_TextAreaHome, true);
    lv_obj_add_flag(ui_TextAreaHome, LV_OBJ_FLAG_HIDDEN);     /// Flags


    ui_KeyboardHome = lv_keyboard_create(ui_Home);
    lv_keyboard_set_mode(ui_KeyboardHome, LV_KEYBOARD_MODE_NUMBER);
    lv_obj_set_width(ui_KeyboardHome, 300);
    lv_obj_set_height(ui_KeyboardHome, 120);
    lv_obj_set_x(ui_KeyboardHome, 0);
    lv_obj_set_y(ui_KeyboardHome, 45);
    lv_obj_set_align(ui_KeyboardHome, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_KeyboardHome, LV_OBJ_FLAG_HIDDEN);     /// Flags

    ui_LbSOS = lv_label_create(ui_Home);
    lv_obj_set_width(ui_LbSOS, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_LbSOS, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_LbSOS, 0);
    lv_obj_set_y(ui_LbSOS, 100);
    lv_obj_set_align(ui_LbSOS, LV_ALIGN_CENTER);
    lv_label_set_text(ui_LbSOS, "SOS ROOM");
    lv_obj_add_flag(ui_LbSOS, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_set_style_text_color(ui_LbSOS, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_LbSOS, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    // lv_obj_add_event_cb(ui_Btn101, ui_event_Btn101, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_BtnAdd, ui_event_BtnAdd, LV_EVENT_ALL, NULL);
    lv_keyboard_set_textarea(ui_KeyboardHome, ui_TextAreaHome);
    lv_obj_add_event_cb(ui_TextAreaHome, ui_event_TextArea, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_KeyboardHome, ui_event_KeyboardHome, LV_EVENT_ALL, NULL);
    
}

