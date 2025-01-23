import time;
import serial;
import matplotlib.pyplot as plt;
import matplotlib.animation as animation;

#configure serial port
ser = serial.Serial('COM8', 9600, timeout=1)
