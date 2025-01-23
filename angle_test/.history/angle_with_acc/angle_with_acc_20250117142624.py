#@Team: ELEC391 B2
#@Author: Jiayi Chen
#@description: To plot the calculated yaw angle using data 
#@Date: 2025/1/17
#@Note: Only measure yaw angle at specific posture, tested algorithm not feasible for use. Measured xy are inverse to what implies on the board.

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

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

#read angle from serial port
def read_angle(ser, prev):
    data = float(ser.readline().rstrip().decode())
    prev = data
    return data

#assign data to variable
def data_gen():
    data_point = data_gen.t
    yaw_angle = data_gen.yaw_angle
    while True:
        data_point += 1
        prev = yaw_angle
        yaw_angle = read_angle(ser, prev)  # replace with your data source

        yield data_point, yaw_angle

#attach data to line
def run(data):
    data_point, y_yaw = data
    if data_point > -1:
        xdata.append(data_point)
        ydata_yaw.append(y_yaw)
        if data_point > xsize:  # Scroll to the left.
            ax.set_xlim(data_point - xsize, data_point)
        line_yaw.set_data(xdata, ydata_yaw)

        # if yaw_angle_text is not None:
        #     yaw_angle_text.remove()

        # yaw_angle_text = ax.text(t, y_yaw, f'({t:.1f}s, {y_yaw:.1f}°)', fontsize=9)

    return line_yaw

#serial initialization
ser = serial_init()

#read data from serial port
data_gen.yaw_angle = read_angle(ser, 0)
data_gen.t = -1

#create plot
xsize = 300
fig, ax = plt.subplots()
line_yaw, = ax.plot([], [], lw=2, label='yaw angle')
ax.set_ylim(-120, 120)
ax.set_xlim(0, xsize)
ax.grid()
ax.legend()
ax.set_title('Measure angle using Acc')
ax.set_xlabel('Data Point')
ax.set_ylabel('Angle')

xdata, ydata_yaw = [], []

#create animation
ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=30, repeat=False)

#show plot
plt.show()