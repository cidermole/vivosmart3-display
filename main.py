import utime
import lvgl as lv

# note: driver and screen initialization was done in boot.py

scr = lv.scr_act()
btn = lv.btn(scr)
btn.align(scr, lv.ALIGN.CENTER, 0, 0)
label = lv.label(btn)

i = 0
t = lv.tick_get()
while True:
    utime.sleep_ms(10)

    if lv.tick_elaps(t) > 100:  # every 100 ms
        i += 1
        t = (t + 100) % (1 << 32)

        label.set_text("Hello i=%d" % i)

        lv.scr_load(scr)
