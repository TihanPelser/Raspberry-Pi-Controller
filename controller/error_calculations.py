import numpy as np


def calculate_errors(vehicle_coords: np.ndarray, vehicle_heading: float, preview_distance: float,
                      path: np.ndarray):
    def line(point_1, point_2):
        a = (point_1[1] - point_2[1])
        b = (point_2[0] - point_1[0])
        c = (point_1[0] * point_2[1] - point_2[0] * point_1[1])
        return a, b, -c

    def intersection(line_1, line_2):
        D = line_1[0] * line_2[1] - line_1[1] * line_2[0]
        Dx = line_1[2] * line_2[1] - line_1[1] * line_2[2]
        Dy = line_1[0] * line_2[2] - line_1[2] * line_2[0]
        if D != 0:
            x = Dx / D
            y = Dy / D
            return round(x, 2), round(y, 2)
        else:
            return None

    preview_delta = preview_distance * np.array([np.cos(vehicle_heading), np.sin(vehicle_heading)])
    preview_point = np.add(vehicle_coords, preview_delta)

    alpha = np.pi / 2 + vehicle_heading
    lateral_extension_delta = np.array([np.cos(alpha), np.sin(alpha)])
    lateral_extension_point = np.add(preview_point, lateral_extension_delta)

    lateral_line = line(point_1=preview_point, point_2=lateral_extension_point)

    lateral_intersect_point = None

    for interval_index in range(len(path) - 1):
        p1 = path[interval_index]
        p2 = path[interval_index + 1]

        test_line = line(point_1=p1, point_2=p2)

        intersect_point = intersection(line_1=lateral_line, line_2=test_line)

        # Check if lines intersect at all
        if intersect_point is None:
            continue

        # Check if lines intersect in segment end points
        if sorted([p1[0], p2[0], intersect_point[0]])[1] == intersect_point[0]:
            sorted_y = sorted([p1[1], p2[1], intersect_point[1]])
            if sorted_y[1] == intersect_point[1]:
                lateral_intersect_point = intersect_point
                break
            else:
                print(f"Y not matching! {sorted_y} vs {intersect_point[1]}")

        # Check if intersect point lies behind final path point
        if interval_index == len(path) - 2:
            intersect_dist = np.sqrt((intersect_point[0] - p1[0]) ** 2 + (intersect_point[1] - p1[1]) ** 2)
            final_point_dist = np.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)
            if intersect_dist > final_point_dist:
                lateral_intersect_point = intersect_point
                if intersect_dist >= 10:
                    return None, None
                break

    if lateral_intersect_point is None:
        return None, None

    lateral_error = np.sqrt((lateral_intersect_point[0] - preview_point[0]) ** 2 +
                            (lateral_intersect_point[1] - preview_point[1]) ** 2)

    # Negative if intersect lies to the left of the vector extending in the direction of the vehicle, and positive
    # if otherwise
    ab = np.array([preview_point[0] - vehicle_coords[0], preview_point[1] - vehicle_coords[1]])
    ac = np.array([lateral_intersect_point[0] - vehicle_coords[0], lateral_intersect_point[1] - vehicle_coords[1]])
    lateral_error_direction = np.sign(np.linalg.det(np.array([ac, ab])))

    path_heading = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])

    heading_error = vehicle_heading - path_heading

    return lateral_error_direction * lateral_error, heading_error
