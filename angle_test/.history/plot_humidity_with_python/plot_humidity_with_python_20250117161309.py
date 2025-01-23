#@team: ELEC391 B2
#@author: Jiayi Chen
#@description: To plot the humidity measured by humidity sensor
#@date: 2025/1/17
#@note: Arduino code use plot_humidity_with_python.ion

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

def serial_init():
    # configure the serial port
    ser = serial.Serial(
        port='COM8', #change this COM port # according to your PC
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_TWO,
        bytesize=serial.EIGHTBITS
    )
    ser.isOpen()
    return ser

#read angle from serial port
def read_humidity(ser, prev):
    data = float(ser.readline().rstrip().decode())
    prev = data
    return data

#assign data to variable
def data_gen():
    data_point = data_gen.t
    humidity = data_gen.humidity
    while True:
        data_point += 1
        prev = humidity
        humidity = read_humidity(ser, prev)  # replace with your data source

        yield data_point, humidity

#attach data to line
def run(data):
    data_point, y_humidity = data
    if data_point > -1:
        xdata.append(data_point)
        ydata_yaw.append(y_humidity)
        if data_point > xsize:  # Scroll to the left.
            ax.set_xlim(data_point - xsize, data_point)
        line_humidity.set_data(xdata, ydata_yaw)

        # if yaw_angle_text is not None:
        #     yaw_angle_text.remove()

        # yaw_angle_text = ax.text(t, y_yaw, f'({t:.1f}s, {y_yaw:.1f}°)', fontsize=9)

    return line_humidity

#serial initialization
ser = serial_init()

#read data from serial port
data_gen.humidity = read_humidity(ser, 0)
data_gen.t = -1

#create plot
xsize = 300
fig, ax = plt.subplots()
line_humidity, = ax.plot([], [], lw=2, label='humidity')
ax.set_ylim(-10, 100)
ax.set_xlim(0, xsize)
ax.grid()
ax.legend()
ax.set_title('Measure Humidity using Humidity Sensor')
ax.set_xlabel('Data Point')
ax.set_ylabel('Humidity')

xdata, ydata_yaw = [], []

#create animation
ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=30, repeat=False)

#show plot
plt.show()