
list(APPEND srcs "lvgl_gui.c"
                "lvgl_adapter.c")
set(incdirs "."
            "include")
idf_component_register(SRCS "${srcs}"
                        INCLUDE_DIRS "${incdirs}"
                        REQUIRES screen touch_panel lvgl)

target_compile_definitions(${COMPONENT_LIB} INTERFACE LV_CONF_INCLUDE_SIMPLE=1)
