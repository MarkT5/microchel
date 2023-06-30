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
line2, = ax.plot(list(range(pltrange)), [0]*pltrange)
line3, = ax.plot(list(range(pltrange)), [0]*pltrange)


plot_data = np.array([[0, 0, 0]])
plot_ang = [[0]]

while True:
    inf = ser.readline()
    if inf:
        out = [list(map(int, inf.decode('utf-8',errors="ignore").split()))]
        plot_data = np.append(plot_data, out, axis=0)
        plot_ang.append(math.atan2(out[0][1], out[0][0])/3.1415*180)
        if len(plot_data)>pltrange:
            plot_data = plot_data[1:]
            plot_ang = plot_ang[1:]
            line1.set_ydata(plot_data[:, 0])
            line2.set_ydata(plot_data[:, 1])
            line3.set_ydata(plot_ang)

            fig.canvas.draw()
            fig.canvas.flush_events()
        #print(math.atan2(out[0][1], out[0][0])/3.1415*180)
        ser.flushInput()
        ser.flushOutput()
