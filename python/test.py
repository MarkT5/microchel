import matplotlib.pyplot as plt
import matplotlib
import serial
import numpy as np
import math


ser = serial.Serial('COM4', 115200, timeout=.1, rtscts=1)
ser.bytesize = 8
ser.stopbits = 1
ser.parity = 'N'

pltrange = 500
matplotlib.use("TkAgg")
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111)
plt.xlim([0, pltrange])
plt.ylim([-400, 400])
line1, = ax.plot(list(range(pltrange)), [0]*pltrange)


plot_ang = [[0]]

while True:
    inf = ser.readline()
    if inf:
        plot_ang.append(float(inf.decode('utf-8',errors="ignore")))
        if len(plot_ang)>pltrange:
            plot_ang = plot_ang[1:]
            line1.set_ydata(plot_ang)
            fig.canvas.draw()
            fig.canvas.flush_events()
        ser.flushInput()
        ser.flushOutput()
