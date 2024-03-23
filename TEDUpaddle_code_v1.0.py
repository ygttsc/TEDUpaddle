import time
# import microcontroller
import supervisor
import board
import busio
import usb_cdc
from pwmio import PWMOut
from digitalio import DigitalInOut, Direction
import struct

serial = usb_cdc.data

if (board.board_id) == "raspberry_pi_pico":
    i2c = busio.I2C(board.GP17, board.GP16, frequency=100000)
    enb = DigitalInOut(board.GP22)
    cw = DigitalInOut(board.GP19)
    ccw = DigitalInOut(board.GP20)
    w = PWMOut(board.GP18, frequency=50000, duty_cycle=0)
elif (board.board_id == "seeeduino_xiao") or (board.board_id == "seeeduino_xiao_rp2040"):
    i2c = busio.I2C(board.D5, board.D4, frequency=100000)
    enb = DigitalInOut(board.D3)
    cw = DigitalInOut(board.D9)
    ccw = DigitalInOut(board.D10)
    w = PWMOut(board.D8, frequency=50000, duty_cycle=0)

enb.direction = Direction.OUTPUT
cw.direction = Direction.OUTPUT
ccw.direction = Direction.OUTPUT

init_flag = False  # Check if the angle is initialized

enb.value = 1  # 1 = motor runs, 0 = motor stops

pi = 3.14
g = 9.81 # gravitational acceleration
a = 0.005 # gram force per bit duty cycle
rh = 50 # handle radius in mm
Vmax = 9 # power supply voltage

dc = 0  # PWM duty cycle 0 to 65535
dcmax = 65535 # maximum duty cycle
pos = 0  # position in degrees
pos_old = 0  # position at the previous sample
vel = 0  # velocity in rad/s
vel_old = 0  # velocity at the previous sample
vel_old_old = 0  # velocity at two samples ago
f = 0.7  # filter coefficient
th = 0  # position variable
k = 0  # stiffness
c = 0  # damping coefficient
d = 0  # virtual wall distance
delta = 0  # distance to the virtual wall
target = 0  # target angular position
deadband = 2000  # deadband to overcome friction
err = 0  # position error
err_old = 0  # position error at the previous sample
err_old_old = 0  # position error at two samples ago
err_der = 0  # derivative of the error
err_int = 0  # integral of the error
K_P = 0  # proportional gain
K_I = 0  # integral gain
K_D = 0  # derivative gain
mode = 1  # mode at start-up
data_in = bytearray(13)
data_out = bytearray(8)

period_comm = 0.02  # serial comm every 20 ms
last_comm = -1  # initialize communication loop time
period_ang = 0.002  # read angle every 2 ms
last_ang = -1  # initialize control loop time
imp_start = 0  # initialize impulse time
impdir = 1

def read_AS5600():  # Subroutine to read angle sensor
    global th
    try:
        i2c.writeto(0x36, bytes([0x0C]))
        data = bytearray(2)
        i2c.readfrom_into(0x36, data)
        ang = data[0] << 8 | data[1]
        th = -ang * 6.28 / 4095
        if init_flag:
            if th < th_min:
                th = th + 6.28 - th_0
            elif th > th_max:
                th = th - 6.28 - th_0
            else:
                th = th - th_0
    finally:
        return round(th, 6)

def set_dir(direction):  # Subroutine to set motor direction
    if direction == "cw":
        cw.value = True
        ccw.value = False
    else:
        cw.value = False
        ccw.value = True

def th_init():  # Subroutine to initialize zero-angle position
    global th_0
    global th_min
    global th_max
    global init_flag
    th_0 = 0
    for i in range(100):
        th_0 += read_AS5600()
        time.sleep(0.01)
    th_0 = th_0 / 100
    th_min = th_0 - 1.57
    th_max = th_0 + 1.57
    print("Initialization complete!")
    init_flag = True

while not i2c.try_lock():
    pass

try:
    while True:
        if not init_flag:
            th_init()

        now = time.monotonic()
        if now >= last_ang + period_ang:    # control loop
            last_ang = now
            pos = read_AS5600()
            vel = -(f*f*vel_old_old)+(2*f*vel_old)+((1-f)*(1-f)*(pos-pos_old)/period_ang)
            #vel = (pos-pos_old)/period_ang

            if mode == 0:
                supervisor.reload()
            elif mode == 1:
                err = target-pos
                err_int = err_int + err
                err_der = (err-err_old)/period_ang
                dc = int(K_P*err + K_I*err_int*period_ang + K_D*err_der)
                err_old_old = err_old
                err_old = err
            elif mode == 2:
                err_int = 0
                dc = -int(k * pos + c * vel)
            elif mode == 3:
                err_int = 0
                delta = abs(pos) - d/2
                if delta < 0:
                    dc = 0
                else:
                    if pos < 0:
                        dc = int(k * delta - c * vel)
                    else:
                        dc = int(-k * delta - c * vel)
            elif mode == 4:
                if (imp_start % int(0.5/period_ang)) < int(0.01/period_ang):
                    dc = (65535 - deadband)*impdir
                else:
                    dc = 0
                imp_start = imp_start + 1
                if imp_start == int(1.5/period_ang):
                    impdir = -impdir
                if imp_start == int(3/period_ang):
                    impdir = -impdir
                    imp_start = 0
                    mode = 1

            if dc > 0:  # set motor direction
                set_dir("cw")
            elif dc < 0:
                set_dir("ccw")
                dc = -dc
            else:
                dc = -deadband

            if dc > (65535 - deadband):  # saturation check
                dc = 65535 - deadband

            w.duty_cycle = dc + deadband    # set duty cycle

            pos_old = pos
            vel_old_old = vel_old
            vel_old = vel

        now = time.monotonic()
        if now >= last_comm + period_comm:  # communication loop
            last_comm = now
            data_out = struct.pack("ff", pos, vel)
            serial.write(data_out)
            if serial.in_waiting > 0:
                data_in = serial.read(13)
                dummy = struct.unpack("bbhhhhhb", data_in)
                mode = dummy[0]
                target = dummy[1]/100 # received target is 100x
                K_P = dummy[2]*dcmax/Vmax/10 # received gains are 10x
                K_I = dummy[3]*dcmax/Vmax/10
                K_D = dummy[4]*dcmax/Vmax/10
                k = dummy[5]*rh/a/g
                c = dummy[6]*rh/a/g/100 # received value is c*100 (float to int)
                d = dummy[7]/rh
            # print(dc, mode, K_P, K_I, K_D, k, c, d)
            # print(c, vel, dc)
finally:  # unlock the i2c bus
    i2c.unlock()


