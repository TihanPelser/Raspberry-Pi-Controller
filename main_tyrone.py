from adafruit_blinka.board import raspi_40pin as board
import busio
from hardware.hardware_controller import HardwareController
from gps_interface.ublox_interface import UBX
import sys, argparse
from geopy.distance import great_circle as gc
import numpy as np


def xyval(pt):
    ycng = [pt[0], origin[1]]
    xcng = [origin[0], pt[1]]
    x = gc(origin, xcng).meters
    y = gc(origin, ycng).meters
    return x, y


origin = [-25.7530795, 28.2275646]
x0, y0 = xyval(origin)

axishift = [-25.75328621, 28.22747239]
xst, yst = xyval(axishift)

theta = np.arctan2((yst - y0), (xst - x0))

xs = round(xst * np.cos(theta) + yst * np.sin(theta), 15)
ys = round(-xst * np.sin(theta) + yst * np.cos(theta), 15)

if __name__ == "__main__":

    point = [-25.75330863, 28.2274733]
    xp, yp = xyval(point)

    xpr = round(xp * np.cos(theta) + yp * np.sin(theta), 15)
    ypr = round(-xp * np.sin(theta) + yp * np.cos(theta), 15)