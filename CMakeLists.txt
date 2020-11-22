set(srcs "wm8960.c"
         "i2c_bus.c"
         "i2s_bus.c")
set(COMPONENT_NAME "wm8960")

idf_component_register(SRCS         "${srcs}"
                       INCLUDE_DIRS "include")
