#need pyserial and matplotlib

import serial
import time
import matplotlib.pyplot as plt

BAUDRATE = 115200
PORT = 'COM7'  # set the correct port before run it

GraphAccuracy = 0.100 # Each data point is about .1 seconds apart.
GraphLength = 150 # Graph is 300 * .1s or 30 seconds.
Graph = [] # Graph data points
GraphPosition = 0

SetpointY = []
LuxY = []
SecondsX = []

plt.ion() # Enable interactive mode.

for i in range(GraphLength):
    Graph.append([0, 0, 0, 0, 0])
    SetpointY.append(0)
    LuxY.append(0)
    SecondsX.append(GraphLength/10 - (i * GraphAccuracy))

z1serial = serial.Serial(port=PORT, baudrate=BAUDRATE)
z1serial.timeout = 2  # set read timeout

# print z1serial  # debug serial.
if z1serial.is_open:
    while True:
        StartTime = time.process_time()
        EndTime = StartTime
        while(EndTime - StartTime < 0.100): # Read data until a tenth of a second passes and use last data obtained.
            size = z1serial.inWaiting()
            if size:
                data = z1serial.readline()
                data = data.decode("ascii")
                data = data.split()
                if(len(data) != 5): # Ignore rows that come in janky.
                    continue
                print(data[0] + ' ' + data[1] + ' ' + data[2] + ' ' + data[3] + ' ' + data[4])
                Graph[GraphLength - 1] = [int(data[0]), int(data[1]), int(data[2]), int(data[3]), int(data[4])]
            EndTime = time.process_time()
        for i in range(1, GraphLength): # Shift graph points.
            Graph[i - 1] = Graph[i]
            SetpointY[i - 1] = Graph[i - 1][0]
            LuxY[i - 1] = Graph[i - 1][1]
        # Plot graph
        plt.cla()
        plt.plot(SecondsX, SetpointY, label = "Setpoint")
        plt.plot(SecondsX, LuxY, label = "Lux")
        plt.xlabel('Seconds')
        plt.ylabel('Lux')
        plt.title('Project 2 Realtime Graph')
        plt.pause(0.01)
else:
    print("z1serial not open")
# z1serial.close()  # close z1serial if z1serial is open.