import serial
import time

arduino = serial.Serial('COM3', 115200, timeout=.1)
time.sleep(2)

while 1:
    time.sleep(0.03)
    data = arduino.readline()
    if data:
        print(str(data))
    datafromUser = input().split()
    arduino.write(datafromUser[0].encode("ascii")+int(datafromUser[1]).to_bytes(2, 'big', signed=True))
    #print(datafromUser[0].encode("ascii")+int(datafromUser[1]).to_bytes(2, 'big', signed=True))