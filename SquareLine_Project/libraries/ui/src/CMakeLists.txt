SET(SOURCES ui_comp_imgbutton1.c
    ui_comp.c
    ui_MainScreen.c
    ui_Screen101.c
    ui_Screen102.c
    ui_Screen201.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_events.c
    ui_img_fire_border_resize_png.c
    ui_img_smoke_border_resize_png.c
    ui_img_back_png.c)

add_library(ui ${SOURCES})
