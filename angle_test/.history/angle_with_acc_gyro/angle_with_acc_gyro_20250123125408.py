#@team: ELEC391 B2
#@author: Jiayi Chen
#@description: To plot the calculated tilt angle using data measured by accelerometer and gyroscope.
#@date: 2025/1/17
#@note: Arduino code use angle_with_acc_gyro.ino

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
def read_angle(ser, prev):
    data = float(ser.readline().rstrip().decode())
    prev = data
    return data

#assign data to variable
def data_generator():
    data_point = data_gen.t
    tilt_angle = data_gen.tilt_angle
    while True:
        data_point += 1
        prev = tilt_angle
        tilt_angle = read_angle(ser, prev)  # replace with your data source

        yield data_point, tilt_angle

#attach data to line, create tilt_angle_text
def run(data):
    data_point, y_tilt = data
    if data_point > -1:
        xdata.append(data_point)
        ydata_tilt.append(y_tilt)
        if data_point > xsize:  # Scroll to the left.
            ax.set_xlim(data_point - xsize, data_point)
        line_tilt.set_data(xdata, ydata_tilt)

        tilt_angle_text_content = "Current Tilt Angle:\n" + "\n".join([f"({x:.2f}, {y:.2f})" for x, y in zip(xdata[-1:], ydata_tilt[-1:])])
        tilt_angle_text.set_text(tilt_angle_text_content)

    return line_tilt, tilt_angle_text_content

#serial initialization
ser = serial_init()

#read data from serial port
data_gen.tilt_angle = read_angle(ser, 0)
data_gen.t = -1

#create plot
xsize = 300
fig, ax = plt.subplots()
line_tilt, = ax.plot([], [], lw=2, label='tilt angle')
ax.set_ylim(-150, 150)
ax.set_xlim(0, xsize)
ax.grid()
ax.legend()
ax.set_title('Measure angle using Accelerometer and Gyroscope')
ax.set_xlabel('Data Point')
ax.set_ylabel('Tilt Angle')
xdata, ydata_tilt = [], []

#create data point text
tilt_angle_text = ax.text(1.05, 0.95, "", fontsize=10, va="top", ha="left", transform=ax.transAxes)
fig.subplots_adjust(right=0.75)

#create animation
ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=70, repeat=False)

#show plot
plt.show()