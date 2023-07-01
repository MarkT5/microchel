import serial
import time
import pygame as pg

arduino = serial.Serial('COM3', 115200, timeout=.1)
time.sleep(2)
# data = arduino.readline()


pg.init()

screen = pg.display.set_mode([500, 500])
pressed_keys = []
move_speed_val = 80
last_checked_pressed_keys = []
grab = False
up = False


def update_keys():
    global last_checked_pressed_keys
    global grab, up
    for event in pg.event.get():
        # Did the user hit a key?
        if event.type == pg.KEYDOWN:
            key = event.key
            if key not in pressed_keys:
                pressed_keys.append(key)
            if event.key == pg.K_ESCAPE:
                running = False
        elif event.type == pg.KEYUP:
            key = event.key
            if key in pressed_keys:
                pressed_keys.pop(pressed_keys.index(key))
    move_speed = [0, 0]
    fov = 0
    if pg.K_w in pressed_keys:
        fov += 1
    if pg.K_s in pressed_keys:
        fov -= 1
    move_speed[0] = fov * move_speed_val

    rot = 0
    if pg.K_a in pressed_keys:
        rot += 1
    if pg.K_d in pressed_keys:
        rot -= 1
    move_speed[1] = rot * move_speed_val
    if last_checked_pressed_keys != pressed_keys:
        last_checked_pressed_keys = pressed_keys[:]
        print(move_speed[0]+move_speed[1],move_speed[0]-move_speed[1])
        arduino.write(b'l' + (move_speed[0]+move_speed[1]).to_bytes(2, 'big', signed=True))
        arduino.write(b'r' + (move_speed[0]-move_speed[1]).to_bytes(2, 'big', signed=True))
        if pg.K_u in pressed_keys:
            arduino.write(b'u' + int(90 + 120 * grab).to_bytes(2, 'big', signed=True))
            print(grab)
            grab = not grab
        if pg.K_g in pressed_keys:
            arduino.write(b'g' + int(110+130*grab).to_bytes(2, 'big', signed=True))
            print(grab)
            grab = not grab


def main_thr():
    while True:
        update_keys()
        pg.display.flip()


while 1:
    time.sleep(0.03)
    main_thr()
    # print(datafromUser[0].encode("ascii")+int(datafromUser[1]).to_bytes(2, 'big', signed=True))
