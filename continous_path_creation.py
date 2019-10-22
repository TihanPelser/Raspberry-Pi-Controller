import numpy as np
from gps_interface.ublox_interface import UBX
from coordinate_conversions import xy
from scipy.interpolate import interp1d
import time
import sys


def create_equidistant_points(points: np.ndarray):
    distance_between_points = 3

    xp = points[:, 0]
    yp = points[:, 1]

    data_x = xp

    f = interp1d(xp, yp, kind="cubic")
    data_y = f(data_x)

    # Linear length on the line
    distance = np.cumsum(np.sqrt(np.ediff1d(data_x, to_begin=0) ** 2 + np.ediff1d(data_y, to_begin=0) ** 2))
    print(f"Before norm: {distance}")
    num_points = round(distance[-1] / distance_between_points)

    distance = distance / distance[-1]
    print(f"After norm: {distance}")

    fx, fy = interp1d(distance, data_x), interp1d(distance, data_y)

    alpha = np.linspace(0, 1, num_points)
    x_regular, y_regular = fx(alpha), fy(alpha)

    # x = np.arange(0, 100.5, 0.5)
    # y = 20 * np.sin(np.radians(3.6*x))
    xy = zip(x_regular, y_regular)

    return np.ndarray(xy)


if __name__ == "__main__":

    PATH_NAME = sys.argv[1]

    gps = UBX()
    gps.start_reading()
    print("Waiting for GPS signal...")
    while gps.lat == 0:
        continue

    while True:
        user_input = input("Press ENTER to start path measurement:")
        if user_input == "":
            break
        else:
            pass

    print("Press CTRL + C to stop logging")
    points = []
    try:
        while True:
            points.append([gps.lat, gps.long])
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("Logging finished!")

    finally:
        points = np.array(points)
        print("Converting points to X Y...")
        origin = points[0]
        path_xy = xy.convert_path(origin=origin, path=points)
        print("Spacing points...")
        spaced_path = create_equidistant_points(points=path_xy)
        print("Writing to file...")
        with open(f"paths/{PATH_NAME}_equidistant.txt", "w+") as file:
            for point in spaced_path:
                file.write(f"{point[0]},{point[1]}\n")
        print("Completed!")
