import numpy as np

import pyproj

geod = pyproj.Geod(ellps='WGS84')

# All the coordinate conversion equations
# Global North is used as the positive X-direction
# Global West is used as the positive Y-direction


# Not really used
def geo_to_xy(origin: np.ndarray, point: np.ndarray):
    dist, theta = calc_distance_and_azimuth(point1=origin, point2=point)
    x = round(dist * np.cos(theta), 2)
    y = round(dist * np.sin(theta), 2)
    return np.array([x, y])


def calc_distance_and_azimuth(point1: np.ndarray, point2: np.ndarray):
    lat_1, long_1 = point1
    lat_2, long_2 = point2
    azimuth_forward, azimuth_backward, distance = geod.inv(long_1, lat_1, long_2, lat_2)
    return distance, np.deg2rad(convert_heading(azimuth_forward))


def convert_path(origin: np.ndarray, path: np.ndarray):
    path_xy = []
    for point in path:
        dist, point_heading = calc_distance_and_azimuth(point1=origin, point2=point)
        # point_heading = np.deg2rad(convert_heading(azi))
        x = round(dist * np.cos(point_heading), 2)
        y = round(dist * np.sin(point_heading), 2)
        path_xy.append([x, y])
    return np.array(path_xy)


def convert_heading(heading: float):
    if heading > 180:
        return 360 - heading
    else:
        return - heading
