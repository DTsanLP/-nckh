// RoomButton.h
#ifndef ROOMBUTTON_H
#define ROOMBUTTON_H

#include <lvgl.h>

typedef struct RoomButton
{
   lv_obj_t* btn;  // Pointer to the button object in LVGL
    int id;         // Unique identifier for the room button
    lv_obj_t* label; // Pointer to the label associated with the button (if needed)
} RoomButton;

#endif // ROOMBUTTON_H
