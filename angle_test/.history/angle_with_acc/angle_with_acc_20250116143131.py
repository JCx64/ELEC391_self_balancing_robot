import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import serial
import ctypes

xsize = 300

def read_init():
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
        line_celsius.set_data(xdata, ydata_yaw)

        if current_celsius is not None:
            current_celsius.remove()
        if current_fahrenheit is not None:
            current_fahrenheit.remove()
        if current_celsius_change is not None:
            current_celsius_change.remove()
        current_celsius = ax.text(t, y_celsius, f'({t:.1f}s, {y_celsius:.1f}°C)', fontsize=9)
        current_fahrenheit = ax.text(t, y_fahrenheit, f'({t:.1f}s, {y_fahrenheit:.1f}°F)', fontsize=9)
        #current_celsius_change = ax.text(t, y_celsius_change, f'({t:.1f}s, {y_celsius_change:.1f}*0.1°C/s)', fontsize=9)

    return line_celsius, line_fahrenheit, line_celsius_change

def on_close_figure(event):
    sys.exit(0)

ignored = False
ser = read_init()
data_gen.val_celsius = read_temp(ser, 0)
data_gen.t = -1
fig, ax = plt.subplots()
fig.canvas.mpl_connect('close_event', on_close_figure)
line_celsius, = ax.plot([], [], lw=2, label='degree Celsius')
ax.set_ylim(-10, 120)
ax.set_xlim(0, xsize)
ax.grid()
ax.legend()

ax.set_title('Measure angle using Acc')
ax.set_xlabel('Time')
ax.set_ylabel('Angle')

xdata, ydata_yaw = [], []

ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=100, repeat=False)
plt.show()
