import time;
import serial;
import matplotlib.pyplot as plt;
import matplotlib.animation as animation;

#configure serial port
ser = serial.Serial('COM8', 9600, timeout=1)

x_data = []
y_data = []
fig, ax = plt.subplot()

line, = ax.plot([], [], lw=2)  # 初始化空图

# 初始化绘图
def init():
    ax.set_xlim(0, 100)  # X 轴范围
    ax.set_ylim(0, 1023)  # Y 轴范围（模拟输入的范围）
    return line,

# 更新绘图
def update(frame):
    data = ser.readline().decode('utf-8').strip()  # 读取串口数据
    if data.isdigit():  # 检查数据是否为数字
        value = int(data)
        y_data.append(value)  # 添加新数据到 Y
        x_data.append(len(x_data))  # X 为数据点序号
        
        # 仅保留最近 100 个点
        if len(x_data) > 100:
            x_data.pop(0)
            y_data.pop(0)
        
        line.set_data(x_data, y_data)  # 更新图形数据
    return line,

# 动画更新
ani = animation(fig, update, init_func=init, blit=True, interval=100)

# 显示图形
plt.show()