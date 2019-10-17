import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from gps_interface.ublox_interface import UBX
from coordinate_conversions import xy
import time
import threading
import numpy as np

style.use('fivethirtyeight')

fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

origin = None

def animate(i):
    global origin
    print("plot" + str(i))
    graph_data = open('datapoints.txt', 'r').read()
    lines = graph_data.split('\n')
    xs = []
    ys = []
    path_lat_long = []
    for line in lines:
        if len(line) > 1:
            x, y = line.split(',')
            path_lat_long.append([float(x), float(y)])
            # xs.append(float(x))
            # ys.append(float(y))
    # print(origin)
    # print(path_lat_long)
    path = xy.convert_path(origin=origin, path=np.array(path_lat_long))
    ax1.clear()
    ax1.scatter(path[:, 0], path[:, 1])


def gps_write():
    while True:
        # print("appending")
        with open("datapoints.txt", "a+") as file:
            file.write(f"{gps.lat},{gps.long}\n")
        time.sleep(1)


if __name__ == "__main__":
    gps = UBX()
    gps.start_reading()
    time.sleep(0.5)
    print("starting")
    # global origin
    origin = np.array([gps.lat, gps.long])
    try:
        write_thread = threading.Thread(target=gps_write, name="gps_writer", daemon=True)
        write_thread.start()
        time.sleep(2)
        print("plotting")
        ani = animation.FuncAnimation(fig, animate, interval=1000)
        plt.show()
    except KeyboardInterrupt:
        exit()
