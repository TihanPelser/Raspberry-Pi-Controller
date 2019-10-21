import matplotlib.pyplot as plt
import numpy as np


if __name__ == "__main__":

    origin = np.array([-25.753004, 28.2276943])
    point1 = np.array([-25.75309977, 28.22762397])

    lat_list = []
    long_list = []
    distance_list = []
    heading_list = []
    x_list = []
    y_list = []
    heading_check_x = []
    heading_check_y = []
    gps_heading_list = []
    converted_head_list =[]

    with open("TihanResults/conversion_test_points.txt", "r") as file:
        for line in file:
            line = line.replace("[", "")
            line = line.replace("]", "")
            lat, long, heading, converted_head, d_to_orig, d_to_p1, head_to_orig, head_to_p1, head_er_orig, head_er_p1 \
                = line.split(",")
            lat_list.append(float(lat))
            long_list.append(float(long))
            distance_list.append(float(d_to_orig))
            heading_list.append(float(head_to_orig))
            
            x = float(d_to_orig) * np.cos(float(head_to_orig))
            y = float(d_to_orig) * np.sin(float(head_to_orig))
            x_list.append(x)
            y_list.append(y)

            x_head = 2 * np.cos(float(converted_head))
            y_head = 2 * np.sin(float(converted_head))

            heading_check_x.append(x + x_head)
            heading_check_y.append(y + y_head)

            gps_heading_list.append(float(heading))
            converted_head_list.append(float(converted_head))

    plt.figure()
    # plt.scatter(x_list, y_list, label="Points")
    # for i in range(len(heading_check_x)):
    #     plt.plot([x_list[i], heading_check_x[i]], [y_list[i], heading_check_y[i]], "g--")
    plt.scatter([i for i in range(len(heading_list))], np.degrees(heading_list), label="Head To Origin")
    plt.scatter([i for i in range(len(converted_head_list))], converted_head_list, label="Converted GPS")
    plt.plot([0, len(converted_head_list)], [180, 180], 'g--')
    plt.plot([0, len(converted_head_list)], [-180, -180], 'g--')

    plt.xlabel("Point")
    plt.ylabel("Angle")
    plt.legend()
    plt.show()
