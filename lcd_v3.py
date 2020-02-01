# Garmin Vivoactive 3 LCD display driver for STM32
#
# MIT license; Copyright (c) 2019 David Madl


# This is a proof-of-concept for the pyboard v1.1 which has an STM32
# without FSMC. Hence, it uses bit-banging to drive the display.
#
# NOTE: This code resets the DWT cycle counter (DWT_CYCCNT) of the STM32
# and uses it to create delays.
#
#
# Usage example:
#
# import lcd_v3
# frame_buf = bytearray(240*240 + 240 + 9*4)
#
# # set up some RGB332 data in frame_buf here
#
# # flush the frame buffer to the LCD display
# lcd_v3.write(frame_buf)
# lcd_v3.pwm_pins_toggle()


# Freeze into micropython for reducing the RAM usage:
#
# * copy to `ports/stm32/modules/lcd_v3.py`
# * in `ports/stm32/boards/manifest.py`, add:
#       freeze('$(MPY_DIR)/ports/stm32/modules', 'lcd_v3.py')
#
# or look up usage of FROZEN_DIR make variable.
#
# For the pyboard v1.1, I then built micropython using:
# $ make -C ports/stm32 LV_CFLAGS="-DLV_COLOR_DEPTH=8" BOARD=PYBV11 all


# chip: STM32F405RGT6
#
# docs:
# * datasheet (DocID022152 Rev 8, https://www.st.com/resource/en/datasheet/stm32f405rg.pdf)
# * reference manual (RM0090 Rev 18, https://www.st.com/resource/en/reference_manual/dm00031020.pdf)
#
# LCD controller chip has no docs available,
# but reverse-engineering results are in the comments below.


import pyb
import stm


#
# # Garmin vivoactive 3 display
#
# Screen resolution is 240x240 pixels.
#
# # Transfer pattern
#
# 4 horizontal pixels  x  2 vertical pixels block:
#
# ```
# {[P_hf T_i] [P_lf T_i] [P_hr T_i] [P_lr T_i]}
# {[P_hf T_o] [P_lf T_o] [P_hr T_o] [P_lr T_o]}
#
# T_i is transfer slots [1,0]
# T_o is transfer slots [3,2]
#
# P_ab
# a: h for pins 21,22,23, l for pins 2,3,4
# b: f for falling edge, r for rising edge
# ```
#
# Each transfer_frame transfers one row's (LSB or MSB) for the entire screen. 
# Each full clock cycle, 4 pixels in {} are transmitted.
#
# After all T_i slots (2 x 61 clks in transfer_frame) of even row,
# the odd row is transmitted with T_o slots (again 2 x 61 clks in transfer_frame).
#
#
# # Memory layout
#
# Framebuffer organization:
#
#
# # Colors
#
# * red: D21 is the left pixel, D4 is the right pixel
# * green: D22 is the left pixel, D3 is the right pixel
# * blue: D23 is the left pixel, D2 is the right pixel
#
#
# # Intensity
#
# T_a, T_b each (both transfer slots: [1,0] and [3,2])
# These are MSB and LSB of color intensity, respectively.
#
#
# # Refresh
#
# Must generate the 60 Hz rectangle on pins 1, 7, 18, so you can see something.
#
#
# # TODOs
#
# * Find the control bit timeslots for backlight brightness.
#
#
# # Color notes
#
# complete area fills:
# (see outer pairs)
#
# D23, D22, D21, D4, D3, D2
#
# 1 0 0 0 0 1  blue (D23, D2) <<<
#
# 1 0 1 1 0 1  pink (D23, D21, D4, D2)
#
# 0 0 1 1 0 0  red (D21, D4) <<<
#
# 1 1 0 0 1 1  lightblue (D23, D22, D3, D2)
#
# 0 1 0 0 1 0  green (D22, D3) <<<
#
# 0 1 1 1 1 0  yellow (D22, D21, D4, D3)
#


# # Implementation
#
# ## Memory layout
#
# Framebuffer organization:
#
# RGB332 (LV_COLOR_DEPTH 8)



# NOTE: must generate the 60 Hz rectangle on pins 1, 7, 18
# (it turns out this toggling is only necessary after a screen update)

# TODO: use pin number constants for the assembly code below
# (currently, the assembly part uses its own HARDCODED pin numbers!)

# data pins
d2  = pyb.Pin('X1', pyb.Pin.OUT_PP)
d3  = pyb.Pin('X2', pyb.Pin.OUT_PP)
d4  = pyb.Pin('X3', pyb.Pin.OUT_PP)
d21 = pyb.Pin('X4', pyb.Pin.OUT_PP)
d22 = pyb.Pin('X5', pyb.Pin.OUT_PP)
d23 = pyb.Pin('X6', pyb.Pin.OUT_PP)

p1  = pyb.Pin('X7', pyb.Pin.OUT_PP)  # ~ pwm_60hz_a
p5  = pyb.Pin('X8', pyb.Pin.OUT_PP)  # CLK
p7  = pyb.Pin('Y9', pyb.Pin.OUT_PP)  # ~ pwm_60hz_not_a
p8  = pyb.Pin('Y10', pyb.Pin.OUT_PP) # CS
p9  = pyb.Pin('Y11', pyb.Pin.OUT_PP) # row
p10 = pyb.Pin('Y12', pyb.Pin.OUT_PP) # END

p16 = pyb.Pin('Y8', pyb.Pin.OUT_PP)  # BEGIN
p17 = pyb.Pin('X9', pyb.Pin.OUT_PP)  # - center
p18 = pyb.Pin('X10', pyb.Pin.OUT_PP) # ~ pwm_60hz_a
p20 = pyb.Pin('X11', pyb.Pin.OUT_PP) # hsync_start
p24 = pyb.Pin('X12', pyb.Pin.OUT_PP) # hsync_end

lcd_pins = [d2, d3, d4, d21, d22, d23, p1, p5, p7, p8, p9, p10, p16, p17, p18, p20, p24]

# OSPEEDR 0b01: medium speed (10 ns fall/rise time, max. 25 MHz), cf. Table 50 of datasheet
OSPEEDR_MEDIUM_SPEED = 0b01
def gpio_set_speed(pin, speed):
    # kudos: https://forum.micropython.org/viewtopic.php?f=6&t=4583
    speed &= 0b11
    ospeedr_addr = (pin.gpio() & 0x7fffffff) + stm.GPIO_OSPEEDR
    ospeedr_shift = pin.pin() * 2
    ospeedr = stm.mem32[ospeedr_addr]
    ospeedr &= ~0b11 << ospeedr_shift
    ospeedr |= speed << ospeedr_shift
    stm.mem32[ospeedr_addr] = ospeedr

for pin in lcd_pins:
    gpio_set_speed(pin, OSPEEDR_MEDIUM_SPEED)

###

green_led = pyb.Pin(pyb.Pin.cpu.A14, pyb.Pin.OUT_PP) # PA14

CLK_SPEED = 168000000

# kudos: https://stackoverflow.com/questions/23612296
DEMCR_TRCENA = 0x01000000
DEMCR = 0xE000EDFC
DWT_CTRL = 0xe0001000
CYCCNTENA = (1<<0)
DWT_CYCCNT = 0xE0001004

# CLK_SPEED = 168000000
def stopwatch_reset():
    # enable DWT
    demcr = stm.mem32[DEMCR]
    stm.mem32[DEMCR] = demcr | DEMCR_TRCENA
    stm.mem32[DWT_CYCCNT] = 0
    # enable CPU cycle counter
    dwt_ctrl = stm.mem32[DWT_CTRL]
    stm.mem32[DWT_CTRL] = dwt_ctrl | CYCCNTENA

stopwatch_reset()

@micropython.asm_thumb
def stopwatch_getticks():
    # get the DWT_CYCCNT address in r1
    #movwt(r1, DWT_CYCCNT)
    movwt(r1, 0xE0001004)
    # load *DWT_CYCCNT = CPU_CYCLES into r0, which is returned

    ldr(r0, [r1,0])

@micropython.asm_thumb
def stopwatch_delay(r0):
    # r0: ticks

    # get the DWT_CYCCNT address in r3
    movwt(r3, 0xE0001004)
    # reset DWT_CYCCNT to zero
    movw(r4, 0)
    str(r4, [r3,0])
    label(delay_loop)
    # load DWT_CYCCNT
    ldr(r4, [r3,0])
    cmp(r4, r0)
    blt(delay_loop)

def pwm_pins():
    """NOTE: instead, use pwm_pins_init() and call pwm_pins_toggle() regularly."""
    # p1, p18 in sync
    # p7 in opposite phase
    phase_delay = CLK_SPEED // 60
    p1.value(1)
    p18.value(1)
    p7.value(0)
    stopwatch_delay(phase_delay / 2)
    p1.value(0)
    p18.value(0)
    p7.value(1)
    stopwatch_delay(phase_delay / 2)


def pwm_pins_init():
    p1.value(1)
    p18.value(1)
    p7.value(0)

def pwm_pins_toggle():
    """call every ~ 17 ms (1 / 60 sec)"""
    # note: seems to be doing some buffer flip in the display(?)
    # (nothing is displayed from pixel data, unless you toggle once.)
    #
    # original application toggles the display with 60 Hz (= stable frame rate?)
    # however, it is enough, after writing pixels, to toggle once.
    p1.value(not p1.value())
    p18.value(not p18.value())
    p7.value(not p7.value())


# asm ref: https://github.com/micropython/micropython/blob/master/docs/reference/
#
# nice-to: named constants:
# MYDATA = const(33)


@micropython.asm_thumb
def write(r0):
    """
    :param r0: bytes array containing the frame buffer, 240x240, 1 byte per pixel (RGB332): [0bRRRGGGBB]
    (120 x 4 + 2) frame transfers, + 8 clks of data
    """
    push({r8})
    mov(r8, r0)  # r8: memory pointer to current pixel data

    b(begin_transmit)

    # def delay_loop(r0)  # r0: ticks;  modifies: r3, r4
    label(delay_loop)

    # get the DWT_CYCCNT address in r3
    movwt(r3, 0xE0001004)
    # reset DWT_CYCCNT to zero
    movw(r4, 0)
    str(r4, [r3,0])
    label(_delay_loop_inner)
    # load DWT_CYCCNT
    ldr(r4, [r3,0])
    cmp(r4, r0)
    blt(_delay_loop_inner)

    bx(lr) # return

    # Timing:
    # * CPU clock: 168 MHz
    # * pixel clock: 1.2 MHz (1:140 clock downscaling)

    #
    # Preamble:
    #

    # * 103 clks of CS
    # * 85 clks of begin
    # before starting the clock

    label(begin_transmit)

    # set CS
    movwt(r1, 0x40020400)
    movw(r2, 1 << 11)
    strh(r2, [r1, 0x00000018])

    # delay (103-85)=18 clks
    movw(r0, 18*140)
    bl(delay_loop)

    # set BEGIN
    movwt(r1, 0x40020400)
    movw(r2, 1 << 15)
    strh(r2, [r1, 0x00000018])

    # delay 80 clks
    movw(r0, 80*140)
    bl(delay_loop)

    # set ROW
    movwt(r1, 0x40020400)
    movw(r2, 1 << 0)
    strh(r2, [r1, 0x00000018])

    # delay 1 clks
    movw(r0, 1*140 - 35)
    bl(delay_loop)

    # in pixel-clocking stage,
    #    if i_clock == 67, on rising edge:
    #        clear BEGIN
    #    if i_clock == (61*4*120 + 128):
    #        clear CS
    #        stop pixel clock

    #
    # Pixel clocking:
    #

    # registers:
    # r0, r3, r4: delay loop
    # r1, r2: gpio
    #
    # r5: bit status of LCD pins  (except for short-lived: HSYNC, CLK)
    # r6: counter 0..61
    # r7: counter 0..480
    # 
    movwt(r5, (1<<16)|(0<<9)|(1<<8)) # 16: BEGIN, 9: ROW, 8: CS
    # use GPIOx_ODR for writing data

    # 1/4 clock periods, with data setup before each clock edge

    movw(r7, 0)

    # phases, with delays:
    #
    #  3     0   1     2   3
    #
    #     D         D         D 
    #     |         |         |
    #           _________    
    #          |         |         |
    # _________|         |_________|
    #        ^
    #        current time

    label(frame_transmit)  # transmit 61 clks of rising and falling bits

    # set HSYNC_START
    movwt(r1, 0x40020800)
    movw(r2, 1 << 4)
    strh(r2, [r1, 0x00000018])

    # conditionally set END
    movw(r4, 120*4-4)
    sub(r4, r7, r4)
    cmp(r4, 0)
    bne(_post_set_end)
    # set END
    movwt(r1, 0x40020400)
    movw(r2, 1 << 1)
    strh(r2, [r1, 0x00000018])
    label(_post_set_end)

    # delay 1/4 clks: phase 0
    movw(r0, 35)
    bl(delay_loop)

    # set CLK
    movwt(r1, 0x40020000)
    movw(r2, 1 << 7)
    strh(r2, [r1, 0x00000018])

    # delay 1/4 clks: phase 1
    movw(r0, 35)
    bl(delay_loop)

    # reset HSYNC_START
    movwt(r1, 0x40020800)
    movw(r2, 1 << 4)
    strh(r2, [r1, 0x0000001A])

    movw(r6, 0)

    # ---- begin pixel_transmit ----

    label(pixel_transmit)
    add(r6, r6, 1)

    # begin setup data bit f_r6

    bl(move_pixel_bits)  # subroutine call

    b(_post_move_pixel_bits)  # skip definition of subroutine


    label(move_pixel_bits)
    # move pixel bits to GPIO (moves r8: data pointer) -- modifies r0, r1, r2, r3, r4

    # reset data
    movwt(r1, 0x40020000)
    movw(r2, 0b111111)
    strh(r2, [r1, 0x0000001A])

    mov(r1, r8)
    ldrh(r2, [r1, 0])  # load two pixels in 16 bits: (right) 0bRRRGGGBB 0bRRRGGGBB (left)

    # whether MSB or LSB of colors, is specified by ROW (see r5, bit 9)

    # MSB v  v   v  v  v   v  bits: 15, 12, 9,  7, 4, 1

    #     1111 1100 0000 0000
    #     5432 1098 7654 3210
    #  0b RRRG GGBB RRRG GGBB    r2: pixel data
    #       right     left

    # for LSB, shift 1 to the left
    movw(r4, (1<<9))
    mov(r0, r5)
    and_(r0, r4)
    cmp(r0, 0)  # check if ROW is even or odd
    bne(_post_data_shift)
    movw(r3, 1)
    lsl(r2, r3)
    label(_post_data_shift)

    movwt(r1, 0x40020000)
    # left pixel: RGB = D21, D22, D23 = A3, A4, A5
    # GPIOA |= (data & (1 << 7)) [>> (7-3)]= [>> 4]
    mov(r0, r2)
    movw(r4, (1<<7))
    and_(r0, r4)
    movw(r4, 4)
    lsr(r0, r4)
    strh(r0, [r1, 0x00000018])
    # GPIOA |= (data & (1 << 4)) [>> (4-4)]= []
    mov(r0, r2)
    movw(r4, (1<<4))
    and_(r0, r4)
    movw(r4, 0)
    lsr(r0, r4)
    strh(r0, [r1, 0x00000018])
    # GPIOA |= (data & (1 << 1)) [>> (1-5)]= [<< 4]
    mov(r0, r2)
    movw(r4, (1<<1))
    and_(r0, r4)
    movw(r4, 4)
    lsl(r0, r4)
    strh(r0, [r1, 0x00000018])
    # right pixel: RGB = D4, D3, D2 = A2, A1, A0
    # GPIOA |= (data & (1 << 15)) [>> (15-2)]= [>> 13]
    mov(r0, r2)
    movw(r4, (1<<15))
    and_(r0, r4)
    movw(r4, 13)
    lsr(r0, r4)
    strh(r0, [r1, 0x00000018])
    # GPIOA |= (data & (1 << 12)) [>> (12-1)]= [>> 11]
    mov(r0, r2)
    movw(r4, (1<<12))
    and_(r0, r4)
    movw(r4, 11)
    lsr(r0, r4)
    strh(r0, [r1, 0x00000018])
    # GPIOA |= (data & (1 << 9)) [>> (9-0)]= [>> 9]
    mov(r0, r2)
    movw(r4, (1<<9))
    and_(r0, r4)
    movw(r4, 9)
    lsr(r0, r4)
    strh(r0, [r1, 0x00000018])

    movw(r2, 2)
    mov(r1, r8)
    add(r1, r1, r2)
    mov(r8, r1)
    # end setup data bit f_r6

    bx(lr) # return
    # end move pixel bits to GPIO

    label(_post_move_pixel_bits)


    # delay 1/4 clks: phase 2
    movw(r0, 35)
    bl(delay_loop)

    # conditionally reset CENTER
    cmp(r6, 42) # at 42 clks
    bne(_post_centeroff_begin)
    # reset CENTER
    movwt(r1, 0x40020400)
    movw(r2, 1 << 6)
    strh(r2, [r1, 0x0000001A])
    label(_post_centeroff_begin)

    # reset CLK
    movwt(r1, 0x40020000)
    movw(r2, 1 << 7)
    strh(r2, [r1, 0x0000001A])

    # delay 1/4 clks: phase 3
    movw(r0, 35)
    bl(delay_loop)

    # conditionally set HSYNC_END
    cmp(r6, 59)
    bne(_post_set_hsync_end)
    movwt(r1, 0x40020800)
    movw(r2, 1 << 5)
    strh(r2, [r1, 0x00000018])
    label(_post_set_hsync_end)

    # conditionally set CENTER
    cmp(r7, 2) # one full frame of 61 clks ...
    blt(_post_center_begin)
    cmp(r6, 21) # ... plus 21 clks
    bne(_post_center_begin)
    # set CENTER
    movwt(r1, 0x40020400)
    movw(r2, 1 << 6)
    strh(r2, [r1, 0x00000018])
    label(_post_center_begin)

    # setup data bit r_r6

    bl(move_pixel_bits)  # subroutine call

    # delay 1/4 clks: phase 0
    movw(r0, 35)
    bl(delay_loop)

    # conditionally reset BEGIN
    cmp(r7, 1) # one full frame of 61 clks ...
    bne(_post_reset_begin)
    cmp(r6, 66-61) # ... plus 5 clks
    bne(_post_reset_begin)
    # reset BEGIN
    movwt(r1, 0x40020400)
    movw(r2, 1 << 15)
    strh(r2, [r1, 0x0000001A])
    label(_post_reset_begin)

    # set CLK
    movwt(r1, 0x40020000)
    movw(r2, 1 << 7)
    strh(r2, [r1, 0x00000018])

    # delay 1/4 clks: phase 1
    movw(r0, 35)
    bl(delay_loop)

    # loop-exit
    movwt(r4, 120*4 + 2)
    sub(r4, r7, r4)
    cmp(r4, 0)
    bne(_post_loop_exit_check)
    cmp(r6, 9)
    beq(loop_exit)
    label(_post_loop_exit_check)

    cmp(r6, 60)
    bge(_post_jump_pixel_transmit)
    b(pixel_transmit)
    label(_post_jump_pixel_transmit)

    # ---- end pixel_transmit ----

    # toggle ROW
    movw(r4, (1<<9))
    eor(r5, r4)
    mov(r0, r5)
    and_(r0, r4)
    cmp(r0, 0)
    beq(_row_set)

    # in addition to resetting ROW, move back r8 pointer by a row (240 bytes), 
    # since we will now write the LSB bits from the same row
    movw(r4, 240)
    mov(r0, r8)
    sub(r0, r0, r4)
    mov(r8, r0)  # r8 = r8 - 240
    #
    movwt(r1, 0x40020400)
    movw(r2, 1 << 0)
    strh(r2, [r1, 0x0000001A])
    b(_post_row)

    label(_row_set)
    movwt(r1, 0x40020400)
    movw(r2, 1 << 0)
    strh(r2, [r1, 0x00000018])

    label(_post_row)

    # delay 1/4 clks: phase 2
    movw(r0, 35)
    bl(delay_loop)

    # reset CLK
    movwt(r1, 0x40020000)
    movw(r2, 1 << 7)
    strh(r2, [r1, 0x0000001A])

    # delay 1/4 clks: phase 3
    movw(r0, 35)
    bl(delay_loop)

    # reset HSYNC_END
    movwt(r1, 0x40020800)
    movw(r2, 1 << 5)
    strh(r2, [r1, 0x0000001A])

    # TODO: exit condition
    # 4x frame = 1 double-row
    # 120 double-rows
    add(r7, r7, 1)
    movwt(r4, 120*4 + 2 + 1)  # +2: (61*2=122 of the last 132 clks), +1: last iteration is broken early
    sub(r4, r4, r7)
    cmp(r4, 0)
    blt(loop_exit)
    b(frame_transmit)

    label(loop_exit)

    # reset CS
    movwt(r1, 0x40020400)
    movw(r2, 1 << 11)
    strh(r2, [r1, 0x0000001A])

    # TODO: double-check below

    # end of last clk pulse

    # delay 1/4 clks: phase 2
    movw(r0, 35)
    bl(delay_loop)

    # reset CLK
    movwt(r1, 0x40020000)
    movw(r2, 1 << 7)
    strh(r2, [r1, 0x0000001A])

    # delay 1/4 clks: phase 3
    movw(r0, 35)
    bl(delay_loop)

    # reset END
    movwt(r1, 0x40020400)
    movw(r2, 1 << 1)
    strh(r2, [r1, 0x0000001A])

    # reset ROW
    movwt(r1, 0x40020400)
    movw(r2, 1 << 0)
    strh(r2, [r1, 0x0000001A])

    mov(r0, r8)
    pop({r8})


# # Outline of frame_transmit:
#
# frame_transmit:
#     phase 0
#     phase 1
#
# pixel_transmit:
#     r6 += 1
#
#     setup_f_r6
#     phase 2
#     phase 3
#     setup_r_r6
#     phase 0
#     phase 1
#
#     while(r6 < 60) # jump pixel_transmit
#
# _post_jump_pixel_transmit:
#     toggle_row
#     phase 2
#     phase 3
#
#     while(r7 < 120*4 + 2 + 1) # jump frame_transmit
#