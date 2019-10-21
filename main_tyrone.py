from adafruit_blinka.board import raspi_40pin as board
import busio
from hardware.hardware_controller import HardwareController
from gps_interface.ublox_interface import UBX
import sys, argparse
from geopy.distance import great_circle as gc
import numpy as np
import time
import math
from typing import Any

SPEED = 1.5

def distance(p1, p2):
    return ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def xyval(pt):
    xcng = [pt[0], ref[1]]
    ycng = [ref[0], pt[1]]
    x = abs(gc(ref, xcng).meters)
    y = abs(gc(ref, ycng).meters)
    return x, y

ref = [-25.74551249, 28.24783167]
xr, yr = xyval(ref)

origin = [-25.74542167, 28.24752052]
xo, yo = xyval(origin)
x0 = xyval(origin)[0] - xo
y0 = xyval(origin)[1] - yo

axishift = [-25.74521202, 28.24748998]
xst = xyval(axishift)[0] - xo
yst = xyval(axishift)[1] - yo

theta = np.arctan((yst - y0)/ (xst - x0))

xs = round(xst*np.cos(theta) + yst*np.sin(theta), 15)
ys = round(-xst*np.sin(theta) + yst*np.cos(theta), 15)

if __name__ == "__main__":

    RUN_NAME = sys.argv[1]

    gps = UBX()
    gps.start_reading()

    hardware_controller = HardwareController(gps)

    ##Path
    pathx = np.arange(0, 20, 0.001)
    pathy = []
    path = []
    for i in pathx:
        pathno = 2
        i = round(i, 3)

        if pathno == 1:
            pathx = []
            R = 15
            t = np.linspace(np.pi, 3 * np.pi, 100000)
            for i in t:
                functx = 7.5 + 0.5 * R * np.cos(-i)
                pathx.append(functx)
                functy = 0.5 * R * np.sin(-i)
                pathy.append(functy)
                path.append((functx, functy))

        if pathno == 2:
            functy = 0

        if pathno == 3:
            functy = 2*np.sin(((2*np.pi)/20)*i)

        pathy.append(functy)    
        path.append((i, functy))

    ##Find initial closest point
    point = [gps.lat, gps.long]

    xp = xyval(point)[0] - xo
    yp = xyval(point)[1] - yo

    x = round(xp * np.cos(theta) + yp * np.sin(theta), 15)
    y = round(-xp * np.sin(theta) + yp * np.cos(theta), 15)

    distvals = []
    pt = [x, y]
    for i in path:
        ptp = i
        distvals.append(distance(pt, ptp))
    indexval = distvals.index(min(distvals))

    k = 2.1

    xlst = x
    ylst = y
    heading = 0

    steering_data = []
    gps_data = []
    xy_list = []
    step = 0
    try:
        # hardware_controller.startup()
        hardware_controller.start_control()
        print("Waiting for gps")
        time.sleep(2)
        print("Starting run")
        time.sleep(2)
        hardware_controller.set_speed(SPEED)
        while True:
            step += 1
            print(f"Step: {step}")
            time.sleep(0.05)
            # gps_data = gps.get_current_data()
            gps_data.append([gps.lat, gps.long, gps.speed, gps.heading, heading])
            st_data = hardware_controller.get_current_data()
            steering_data.append([st_data.steering_angle_set_point, st_data.current_steering_angle])
            velocity = gps.speed
            ##Convert XY
            point = [gps.lat, gps.long]

            xp = xyval(point)[0] - xo
            yp = xyval(point)[1] - yo

            x = round(xp * np.cos(theta) + yp * np.sin(theta), 15)
            y = round(-xp * np.sin(theta) + yp * np.cos(theta), 15)

            xy_list.append([x, y])

            ##Adjust heading
            #heading = np.arctan2((y-ylst), (x - xlst))                ##Use if GPS Heading is unreliable
            #xlst = x                                                  ##Use if GPS Heading is unreliable
            #ylst = y                                                  ##Use if GPS Heading is unreliable

            heading = math.degrees(theta) - gps.heading - 90           ##Use if GPS Heading is reliable
            heading = math.radians(heading)                            ##Use if GPS Heading is reliable
            heading = heading%math.radians(360)                        ##Use if GPS Heading is reliable
            if heading > math.radians(180):                            ##Use if GPS Heading is reliable
                heading = heading - math.radians(360)                  ##Use if GPS Heading is reliable

            #finding smallest distance between point and centre of gravity
            distvals = []
            up = True                                                   ##Change to false if error occurs in line 119 - 137

            if up == True:
                pt = [x, y]
                # lkdist = int(50 * (velocity * 3.6))
                lkdist = 75
                if indexval - lkdist < 0:
                    pathc = path[0:(indexval + lkdist)]
                elif indexval + lkdist > len(path) - 1:
                    pathc = path[indexval - lkdist:-1]
                else:
                    pathc = path[indexval - lkdist:(indexval + lkdist)]
                if indexval == len(path):
                    indexval = 0
                for i in pathc:
                    ptp = i
                    distvals.append(distance(pt, ptp))
                mind = distvals.index(min(distvals))
                if indexval - lkdist < 0:
                    indexval = indexval + mind
                else:
                    indexval = indexval - lkdist + mind

            if up == False:
                pt = [x, y]
                path = np.column_stack((pathx, pathy))
                for i in path:
                    ptp = [i[0], i[1]]
                    distvals.append(distance(pt, ptp))
                indexval = distvals.index(min(distvals))

            derr = np.sqrt(min(distvals))

            ##path angle
            thetap = np.arctan((pathy[indexval] - pathy[indexval-1])/(pathx[indexval] - pathx[indexval-1]))

            #angle between steering and path
            if pathx[indexval] < pathx[indexval-1]:
                thetap = thetap - math.radians(180)
            error = - (heading - thetap) % math.radians(360)
            if error > math.radians(180):
                error = error - math.radians(360)

            #steering angle adjustment
            M = np.tan(thetap)
            ysam = M*(x - pathx[indexval]) + pathy[indexval]
            if math.radians(-90) < thetap < math.radians(90):
                if y > ysam:
                    derr = -derr
            if math.radians(90) < thetap < math.radians(180):
                if y < ysam:
                    derr = -derr
            if math.radians(-90) > thetap > math.radians(-180):
                if y < ysam:
                    derr = -derr

            if -0.005 < derr < 0.005:
                d_f = error
            else:
                d_f = error + np.arctan(k * derr / velocity)
                
            if d_f > math.radians(20):
                d_f = math.radians(20)
            elif d_f < math.radians(-20):
                d_f = math.radians(-20)

            d_f = math.degrees(d_f)

            hardware_controller.set_steering_angle(d_f)

            if indexval >= len(path)-10:
                break

    except KeyboardInterrupt:
        hardware_controller.stop_control()

    print("Saving data")
    with open(f"TyroneResults/{RUN_NAME}_gps.txt", "w+") as file:
        for points in gps_data:
            file.write(f"{points[0]},{points[1]},{points[2]},{points[3]},{points[4]}\n")
    with open(f"TyroneResults/{RUN_NAME}_steering.txt", "w+") as file:
        for f in steering_data:
            file.write(f"{f[0]},{f[1]}\n")
    with open(f"TyroneResults/{RUN_NAME}_xy.txt", "w+") as file:
        for points in xy_list:
            file.write(f"{points[0]},{points[1]}\n")
