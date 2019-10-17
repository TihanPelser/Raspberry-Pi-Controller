from gps_interface.ublox_interface import UBX
import sys, argparse
from coordinate_conversions import xy
import numpy as np
import time

if __name__ == "__main__":

    gps = UBX()
    gps.start_reading()
    time.sleep(2)

    origin = None
    point1 = None
    results = []

    while True:
        user_input = input("Enter to set origin:")
        if user_input == "":
            origin = np.array([gps.lat, gps.long])
            break
        else:
            pass

    while True:
        user_input = input("Enter to set first point")
        if user_input == "":
            point1 = np.array([gps.lat, gps.long])
            break
        else:
            pass
    try:
        print("GPSHead\tConvHead\tD_Orig\tD_P1\tH_Orig\tH_P1\tH_Er_Orig\tH_Er_P1")
        while True:
            time.sleep(0.5)
            current_coord = np.array([gps.lat, gps.long])
            heading = round(gps.heading, 4)
            converted_head = round(xy.convert_heading(heading), 4)

            d_to_orig, head_to_orig = xy.calc_distance_and_azimuth(point1=origin, point2=current_coord)
            d_to_p1, head_to_p1 = xy.calc_distance_and_azimuth(point1=point1, point2=current_coord)

            head_er_orig = round(head_to_orig - converted_head, 4)
            head_er_p1 = round(head_to_p1 - converted_head, 4)
            d_to_orig = round(d_to_orig, 4)
            d_to_p1 = round(d_to_p1, 4)
            head_to_p1 = round(head_to_p1, 4)
            head_to_orig = round(head_to_orig, 4)
            results.append([gps.lat, gps.long, heading, converted_head, d_to_orig, d_to_p1, head_to_orig, head_to_p1, head_er_orig, head_er_p1])
            print("\r")
            print(f"\r{heading}\t{converted_head}\t{d_to_orig}\t{d_to_p1}\t{head_to_orig}\t{head_to_p1}\t{head_er_orig}\t{head_er_p1}")
    except KeyboardInterrupt:
        print("Done")

    with open("TihanResults/conversion_test_points.txt", "w+") as file:
        file.write(origin + "\n")
        file.write(point1 + "\n")
        for point in results:
            file.write(str(point) + "\n")
