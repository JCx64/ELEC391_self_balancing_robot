import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import serial
import ctypes

xsize = 300

def serial_init():
    # configure the serial port
    ser = serial.Serial(
        port='COM8',
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS
    )
    ser.isOpen()
    return ser

def read_angle(ser, prev):
    data = float(ser.readline().rstrip().decode())
    prev = data
    return data

def data_gen():
    t = data_gen.t
    yaw_angle = data_gen.yaw_angle
    while True:
        t += 1
        prev = yaw_angle
        yaw_angle = read_angle(ser, prev)  # replace with your data source

        yield t, yaw_angle

def run(data):
    t, y_yaw = data
    if t > -1:
        xdata.append(t)
        ydata_yaw.append(y_yaw)
        if t > xsize:  # Scroll to the left.
            ax.set_xlim(t - xsize, t)
        line_yaw.set_data(xdata, ydata_yaw)

        if yaw_angle is not None:
            yaw_angle.remove()

        yaw_angle = ax.text(t, y_yaw, f'({t:.1f}s, {y_yaw:.1f}°)', fontsize=9)

    return line_yaw

def on_close_figure(event):
    sys.exit(0)

#serial initialization
ser = serial_init()

#read data from serial port
data_gen.yaw_angle = read_angle(ser, 0)
data_gen.t = -1

#create plot
fig, ax = plt.subplots()
line_yaw, = ax.plot([], [], lw=2, label='yaw angle')
ax.set_ylim(-120, 120)
ax.set_xlim(0, xsize)
ax.grid()
ax.legend()
ax.set_title('Measure angle using Acc')
ax.set_xlabel('Time')
ax.set_ylabel('Angle')

xdata, ydata_yaw = [], []

#
ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=100, repeat=False)
plt.show()
