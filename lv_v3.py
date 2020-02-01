# Garmin Vivoactive 3 LCD display driver for STM32 and LittlevGL
#
# MIT license; Copyright (c) 2019 David Madl

import lvgl as lv
import lcd_v3

# should start with this, otherwise there may not be enough memory left
frame_buf = bytearray(240*240 + 240 + 9*4)

def my_flush_cb(disp_drv, area, color_p):
    lcd_v3.write(frame_buf)
    lcd_v3.pwm_pins_toggle()  # note: original application toggles the display with 60 Hz (= stable frame rate?)
    lv.disp_flush_ready(disp_drv)

# Display buffer
disp_buf = lv.disp_buf_t()
lv.disp_buf_init(disp_buf, frame_buf, None, 240*240)

# Display driver
disp_drv = lv.disp_drv_t()
lv.disp_drv_init(disp_drv)
disp_drv.buffer = disp_buf
disp_drv.flush_cb = my_flush_cb
disp_drv.hor_res = 240
disp_drv.ver_res = 240
disp = lv.disp_drv_register(disp_drv)

lcd_v3.pwm_pins_init()
