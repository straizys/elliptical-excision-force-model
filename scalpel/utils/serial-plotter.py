import serial
from collections import deque
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class AnalogPlot:

    def __init__(self, strPort, maxLen):

        self.ser = serial.Serial(strPort, 9600)

        # device initialization
        self.ser.write(b'2')  # start the stream
        line = self.ser.readline()  # ignore first 100 msgs...
        print(line)

        self.sensor_1 = deque([0.0] * maxLen)
        self.sensor_2 = deque([0.0] * maxLen)
        self.sensor_diff = deque([0.0] * maxLen)
        self.maxLen = maxLen

        self.run = False
        self.base = 0
        self.trackPause = False

    # add to buffer
    def addToBuf(self, buf, val):
        if len(buf) < self.maxLen:
            buf.append(val)
        else:
            buf.pop()
            buf.appendleft(val)

    # add data
    def add(self, data):
        assert (len(data) == 2)
        self.addToBuf(self.sensor_1, int(data[0][1:], 16))
        self.addToBuf(self.sensor_2, int(data[1][1:], 16))

        diff = int(data[0][1:], 16) - int(data[1][1:], 16)
        data = self.track_base(diff)

        self.addToBuf(self.sensor_diff, data)

    def track_base(self, datain):

        if (datain - self.base) > 3000:
            self.trackPause = True
        else:
            self.trackPause = False

        if not self.run:
            self.base = datain
            self.run = True
        else:
            if not self.trackPause:
                if datain > self.base:
                    self.base += 300
                else:
                    self.base -= 300

        data = datain - self.base

        return data

    # update plot
    def update(self, frameNum, axis):
        line = self.ser.readline()
        data = [val for val in line.split()]
        # print(data)
        self.add(data[:2])
        axis.set_data(range(self.maxLen), self.sensor_diff)

        return axis

    # close serial and clean up
    def close(self):
        self.ser.flush()
        self.ser.close()

def main():

    strPort = 'COM5'
    analogPlot = AnalogPlot(strPort, maxLen=100)

    fig = plt.figure()
    ax = plt.axes(xlim=(0, 100), ylim=(-500, 50000))
    axis, = ax.plot([], [])
    anim = animation.FuncAnimation(fig, analogPlot.update,
                                   fargs=(axis, ),
                                   interval=1)
    plt.show()

    # clean up
    analogPlot.close()

if __name__ == '__main__':
    main()