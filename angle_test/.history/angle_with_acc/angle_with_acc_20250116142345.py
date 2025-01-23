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
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS
    )
    ser.isOpen()
    return ser

def read_temp(ser, prev):
    data = float(ser.readline().rstrip().decode())
    prev = data
    return data
def Mbox(title, text, style):
    return ctypes.windll.user32.MessageBoxW(0, text, title, style)

def show_alert():
    global ignored
    choice = Mbox('Temperature alert!', 'Temperature is too high!', 2)
    match choice:
        case 3:
            ser.close()
            sys.exit(0)
        case 5:
            ignored = True

def celsius_to_fahrenheit(celsius):
    return celsius * 9 / 5 + 32


def data_gen():
    global ignored
    t = data_gen.t
    val_celsius = data_gen.val_celsius
    while True:
        t += 1
        prev = val_celsius
        val_celsius = read_temp(ser, prev)  # replace with your data source
        if (val_celsius > 40) and not(ignored):
            show_alert()
        if (val_celsius < 40) and (ignored):
            ignored = False
        val_fahrenheit = celsius_to_fahrenheit(val_celsius)
        celsius_change = 10*(val_celsius - prev)
        yield t, val_celsius, val_fahrenheit, celsius_change

current_celsius = None
current_fahrenheit = None
current_celsius_change = None

def run(data):
    global current_celsius, current_fahrenheit, current_celsius_change
    t, y_celsius, y_fahrenheit, y_celsius_change = data
    if t > -1:
        xdata.append(t)
        ydata_celsius.append(y_celsius)
        ydata_fahrenheit.append(y_fahrenheit)
        ydata_celsius_change.append(y_celsius_change)
        if t > xsize:  # Scroll to the left.
            ax.set_xlim(t - xsize, t)
        line_celsius.set_data(xdata, ydata_celsius)
        line_fahrenheit.set_data(xdata, ydata_fahrenheit)
        #line_celsius_change.set_data(xdata, ydata_celsius_change)

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
line_fahrenheit, = ax.plot([], [], lw=2, label='degree Fahrenheit')
line_celsius_change, = ax.plot([], [], lw=2, label='*0.1 degree Celsius per second')
ax.set_ylim(-10, 120)
ax.set_xlim(0, xsize)
ax.grid()
ax.legend()

ax.set_title('LM 335 Temperature')
ax.set_xlabel('Time')
ax.set_ylabel('Temperature')

xdata, ydata_celsius, ydata_fahrenheit, ydata_celsius_change = [], [], [], []

ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=100, repeat=False)
plt.show()
