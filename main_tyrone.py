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

def distance(p1, p2):
    return ((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

origin = [-25.75298435, 28.22771581]
x0, y0 = xyval(origin)

axishift = [-25.75319658, 28.22755953]
xst, yst = xyval(axishift)

theta = np.arctan2((yst - y0), (xst - x0))

xs = round(xst * np.cos(theta) + yst * np.sin(theta), 15)
ys = round(-xst * np.sin(theta) + yst * np.cos(theta), 15)

if __name__ == "__main__":

    gps_results_file = sys.argv[1]
    starget_results_file = sys.argv[1]

    ##Path
    pathx = np.arange(0, 35, 0.001)
    pathy = []
    path = []
    for i in pathx:
        i = round(i, 3)
        if i <30:
            functy = 0
        else:
            functy = -0.015625/2*(i-30)**4
        pathy.append(functy)    
        path.append((i, functy))

    indexval = 0
    k = 5

    try:
        hardware_controller.startup()
        hardware_controller.start_control()
        postiton_data = []
        steering_data = []
        while True:
            gps_data = gps.get_current_data()
            gps_data.append([gps.lat, gps.long, gps.speed, gps.heading])
            st_data = hardware_controller.get_current_data()
            steering_data.append(st_data.steering_angle_set_point, st_data.current_steering_angle)

            steering.append(d_f)

            ##Convert XY
            point = [gps.lat, gps.long]
            xp, yp = xyval(point)

            x = round(xp * np.cos(theta) + yp * np.sin(theta), 15)
            y = round(-xp * np.sin(theta) + yp * np.cos(theta), 15)

            ##Adjust heading
            heading = gps.heading - theta
            heading = heading%math.radians(360)
            if heading>math.radians(180):
                heading = heading - math.radians(360)

            #finding smallest distance between point and centre of gravity
            pt = [x, y]
            lkdist = int(50*(velocity*3.6))
            pathc = path[indexval:(indexval+lkdist)]
            distvals = []
            indexval = indexval + 1
            for i in pathc:
                ptp = i
                distvals.append(distance(pt, ptp))
            mind = distvals.index(min(distvals))        
            indexval = mind + indexval + 1
            if indexval>(len(path)-1):
                indexval = (len(path)-1)
            derr = np.sqrt(min(distvals))

            ##path gradient
            thetap = np.arctan((pathy[indexval] - pathy[indexval-1])/(pathx[indexval] - pathx[indexval-1]))

            #angle between steering and path
            if pathx[indexval]<pathx[indexval-1]:
                thetap = thetap - math.radians(180)
            error = -(heading - thetap)%math.radians(360)
            if error>math.radians(180):
                error = error - math.radians(360)

            #steering angle adjustment
            M = np.tan(thetap)
            ysam = M*(x - pathx[indexval]) + pathy[indexval]
            if pathx[indexval]>pathx[indexval-1]:
                if math.radians(-90) < thetac < math.radians(90):
                    if y>ysam:
                        derr = -derr
                if math.radians(-90) > thetac > math.radians(90):
                    if y<ysam:
                        derr = -derr
            elif pathx[indexval]<pathx[indexval-1]:
                if math.radians(-90) < thetac < math.radians(90):
                    if y<ysam:
                        derr = -derr
                if math.radians(-90) > thetac > math.radians(90):
                    if y>ysam:
                        derr = -derr

            if -0.005 < derr < 0.005:
                d_f = error
            else:
                d_f = error + np.arctan(k*derr/gps.speed)
                
            if d_f > math.radians(35):
                d_f = math.radians(35)
            elif d_f < math.radians(-35):
                d_f = math.radians(-35)

            d_f = math.degrees(d_f)

            hardware_controller.set_steering_angle(d_f)

            if indexval == len(path)-1:
                break

    except KeyboardInterrupt:
        hardware_controller.shutdown()

    with open(f"TyroneResults/{gps_results_file}.txt", "w+") as file:
        for points in postion_data:
            output.write(str(points) + "\n")
    with open(f"TyroneResults/{steering_results_file}.txt", "w+") as file:
        for f in steering_data:
            output.write(str(f) + "/n")