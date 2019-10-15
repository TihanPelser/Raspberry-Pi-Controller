import numpy as np
from geopy.distance import great_circle as gc

# All the coordinate conversion equations


def geo_to_xy(origin: tuple, point: tuple):
    delta_y = [point[0], origin[1]]
    delta_x = [origin[0], point[1]]
    x = gc(origin, delta_x).meters
    y = gc(origin, delta_y).meters
    return x, y


def convert_path(origin: tuple, path: list):
    path_xy = []
    for point in path:
        point_xy = geo_to_xy(origin=origin, point=point)
        path_xy.append(point_xy)
    return path_xy
