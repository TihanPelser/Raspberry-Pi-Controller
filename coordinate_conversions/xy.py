import numpy as np
from geopy.distance import great_circle as gc

# All the coordinate conversion equations
# Global North is used as the positive X-direction
# Global West is used as the positive Y-direction


# Not really used
def geo_to_xy(origin: np.ndarray, point: np.ndarray):
    delta_y = [point[0], origin[1]]
    delta_x = [origin[0], point[1]]
    x = gc(origin, delta_x).meters
    y = gc(origin, delta_y).meters
    return x, y


def calculate_distance(point1: np.ndarray, point2: np.ndarray):
    dist = gc(point1, point2).meters
    return dist


def calculate_azimuth(point1: np.ndarray, point2: np.ndarray):
    delta_lat = point2[0] - point1[0]
    delta_long = point2[1] - point1[1]
    azimuth = np.arctan2((np.sin(delta_long) * np.cos(point2[0])),
                         (np.cos(point1[0]) * np.sin(point2[0]) -
                          np.sin(point1[0] * np.cos(point2[0]) * np.cos(delta_lat))))
    azimuth = np.rad2deg(azimuth)
    return azimuth


# def convert_path(origin: np.ndarray, path: np.ndarray):
#     path_xy = []
#     for point in path:
#         point_xy = geo_to_xy(origin=origin, point=point)
#         path_xy.append(point_xy)
#     return path_xy

def convert_path(origin: np.ndarray, path: np.ndarray):
    path_xy = []
    for point in path:
        point_dist = calculate_distance(point1=origin, point2=point)
        point_azimuth = calculate_azimuth(point1=origin, point2=point)
        x = point_dist * np.cos(point_azimuth)
        y = point_dist * np.sin(point_azimuth)
        path_xy.append([x, y])
    return np.array(path_xy)


def convert_heading(heading: float):
    if heading > 180:
        return 360 - heading
    else:
        return - heading
