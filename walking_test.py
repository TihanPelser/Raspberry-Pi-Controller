# MAIN FILE TO BE EXECUTED
# from adafruit_blinka.board import raspi_40pin as board
# import busio
# from hardware.hardware_controller import HardwareController
from gps_interface.ublox_interface import UBX
import sys, argparse
from coordinate_conversions import xy
import numpy as np
import time

# PATH CONTROLLER
from controller.DQNController import DQNController

INPUT_PATH_FILE = "paths/straight_test.txt"
SPEED_SET_POINT = 0
WAY_POINT_THRESHOLD = 1

def read_path(path_file: str):
    path_data = []
    with open(path_file, "r") as file:
        for line in file:
            x, y = line.split(",")
            path_data.append([x, y])

    return np.array(path_data)


if __name__ == "__main__":

    RUN_NAME = sys.argv[1]

    path = read_path(INPUT_PATH_FILE)

    gps = UBX()
    # I2C Setup
    # i2c = busio.I2C(board.SCL, board.SDA)

    gps.start_reading()

    # hardware_controller = HardwareController(gps=gps)

    path_controller = DQNController(model_file="controller/models/COMPLEX_ARCH_1_IN_5_OUT.h5")
    print("Waiting for GPS signal...")
    time.sleep(2)
    origin = np.array([gps.lat, gps.long])

    path_xy = xy.convert_path(origin=origin, path=path)

    gps_results = []
    hardware_results = []
    calculation_results = []
    dqn_results = []

    end_reached = False
    current_way_point_index = 0
    current_way_point_coords = path[current_way_point_index]
    try:
        # hardware_controller.start_control()

        # Speed Override:
        # speed_volt = 0.8
        # hardware_controller.speed_dac.value = int(round(speed_volt/5) * 65535)
        print("Starting run!")
        while not end_reached:
            # LOGGING
            # hardware_results.append(hardware_controller.get_current_data())
            gps_results.append(gps.get_current_data())

            # UPDATE PARAMETERS
            current_heading = xy.convert_heading(gps.heading)
            current_coords = np.array([gps.lat, gps.long])
            distance_to_next, heading_to_next = xy.calc_distance_and_azimuth(point1=current_coords,
                                                                             point2=current_way_point_coords)
            calculation_results.append([current_heading, distance_to_next, heading_to_next])

            # UPDATE WAY POINTS
            if distance_to_next <= WAY_POINT_THRESHOLD:
                current_way_point_index += 1
                if current_way_point_index == len(path) - 1:
                    end_reached = True
                    break
                else:
                    current_way_point_coords = path[current_way_point_index]

            heading_error = heading_to_next - current_heading

            input_state = np.array([heading_error / 120])
            input_state = np.reshape(input_state, [1, 1])

            dqn_action = path_controller.act(input_state)

            dqn_results.append([input_state[0, 0], dqn_action])

            # hardware_controller.set_steering_angle(dqn_action)

            time.sleep(0.1)

    except KeyboardInterrupt:
        # hardware_controller.stop_control()
        pass
    print("Saving data...")

    with open(f"TihanResults/{RUN_NAME}_gps.txt", "w+") as file:
        for point in gps_results:
            file.write(str(point) + "\n")

    # with open(f"TihanResults/{RUN_NAME}_hw.txt", "w+") as file:
    #     for point in hardware_results:
    #         file.write(str(point) + "\n")

    with open(f"TihanResults/{RUN_NAME}_calc.txt", "w+") as file:
        for point in calculation_results:
            file.write(str(point) + "\n")

    with open(f"TihanResults/{RUN_NAME}_dqn.txt", "w+") as file:
        for point in dqn_results:
            file.write(str(point) + "\n")

    print("Data saved!")
