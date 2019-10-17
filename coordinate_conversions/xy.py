import numpy as np
from geopy.distance import great_circle as gc
import pyproj

geod = pyproj.Geod(ellps='WGS84')

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


def haversine(point1: np.ndarray, point2: np.ndarray):
    r = 6372800  # Earth radius in meters
    lat1, lon1 = point1
    lat2, lon2 = point2

    phi1, phi2 = np.deg2rad(lat1), np.deg2rad(lat2)
    dphi = np.deg2rad(lat2 - lat1)
    dlambda = np.deg2rad(lon2 - lon1)

    a = np.sin(dphi / 2) ** 2 + \
        np.cos(phi1) * np.cos(phi2) * np.sin(dlambda / 2) ** 2

    return 2 * r * np.arctan2(np.sqrt(a), np.sqrt(1 - a))


def calc_distance_and_azimuth(point1: np.ndarray, point2: np.ndarray):
    lat_1, long_1 = point1
    lat_2, long_2 = point2
    azimuth_forward, azimuth_backward, distance = geod.inv(long_1, lat_1, long_2, lat_2)
    return distance, azimuth_forward


def calculate_distance(point1: np.ndarray, point2: np.ndarray):
    dist = gc(point1, point2).meters
    return dist


def calculate_azimuth(point1: np.ndarray, point2: np.ndarray):
    lat1 = point1[0]
    lat2 = point2[0]
    long1 = point1[1]
    long2 = point2[1]
    delta_lat = lat2 - lat1
    delta_long = long2 - long1
    azimuth = np.arctan2((np.sin(delta_long) * np.cos(lat2)),
                         (np.cos(lat1) * np.sin(lat2) -
                          np.sin(lat1 * np.cos(lat2) * np.cos(delta_long))))
    azimuth = np.rad2deg(azimuth)
    azimuth = (azimuth + 360) % 360
    return azimuth


# def convert_path(origin: np.ndarray, path: np.ndarray):
#     path_xy = []
#     for point in path:
#         point_xy = geo_to_xy(origin=origin, point=point)
#         path_xy.append(point_xy)
#     return path_xy

def convert_path(origin: np.ndarray, path: np.ndarray):
    # path_xy = []
    # for point in path:
    #     point_dist = calculate_distance(point1=origin, point2=point)
    #     point_azimuth = calculate_azimuth(point1=origin, point2=point)
    #     point_heading = np.deg2rad(convert_heading(point_azimuth))
    #     x = point_dist * np.cos(point_heading)
    #     y = point_dist * np.sin(point_heading)
    #     path_xy.append([x, y])
    # return np.array(path_xy)
    path_xy = []
    for point in path:
        dist, azi = calc_distance_and_azimuth(point1=origin, point2=point)
        point_heading = np.deg2rad(convert_heading(azi))
        x = dist * np.cos(point_heading)
        y = dist * np.sin(point_heading)
        path_xy.append([x, y])
    return np.array(path_xy)


def convert_heading(heading: float):
    if heading > 180:
        return 360 - heading
    else:
        return - heading
