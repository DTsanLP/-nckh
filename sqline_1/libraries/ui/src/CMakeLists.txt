SET(SOURCES ui_Home.c
    ui_Room.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    ui_img_back_png.c
    ui_img_fire_border_resize_png.c
    ui_img_smoke_border_resize_png.c)

add_library(ui ${SOURCES})
