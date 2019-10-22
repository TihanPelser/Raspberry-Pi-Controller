# MAIN FILE TO BE EXECUTED
from adafruit_blinka.board import raspi_40pin as board
import busio
from hardware.hardware_controller import HardwareController
from gps_interface.ublox_interface import UBX
import sys, argparse
from coordinate_conversions import xy
import numpy as np
import time

# PATH CONTROLLER
from controller.DQNController import DQNController
from controller.error_calculations import calculate_errors

INPUT_PATH_FILE = "paths/straight.txt"
SPEED_SET_POINT = 0
WAY_POINT_THRESHOLD = 0.5

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

    hardware_controller = HardwareController(gps=gps)

    path_controller = DQNController(model_file="controller/models/2_IN_5_OUT_PATH_FINAL.h5")
    print("Waiting for GPS signal...")
    time.sleep(2)
    origin = np.array([gps.lat, gps.long])
    path = np.vstack([origin, path])
    path_xy = xy.convert_path(origin=origin, path=path)

    gps_results = []
    hardware_results = []
    calculation_results = []
    dqn_results = []

    end_reached = False
    try:
        hardware_controller.start_control()
        hardware_controller.set_speed(1.5)
        time.sleep(1.5)
        # Speed Override:
        # speed_volt = 0.8
        # hardware_controller.speed_dac.value = int(round(speed_volt/5) * 65535)
        print("Starting run!")
        while not end_reached:
            # LOGGING
            hardware_results.append(hardware_controller.get_current_data())
            gps_results.append(gps.get_current_data())

            # UPDATE PARAMETERS
            current_heading = np.deg2rad(xy.convert_heading(gps.heading))
            current_coords = np.array([gps.lat, gps.long])
            converted_coords = xy.geo_to_xy(origin=origin, point=current_coords)
            lat_error, yaw_error = calculate_errors(vehicle_coords=converted_coords, vehicle_heading=current_heading,
                                                    preview_distance=2.78, path=path_xy)
            distance_to_end, _ = xy.calc_distance_and_azimuth(point1=current_coords, point2=path[-1])

            print("Current Data:")
            print(f"Lat Err = {lat_error} || Yaw Err = {yaw_error} || Distance To End = {distance_to_end}")

            if distance_to_end <= WAY_POINT_THRESHOLD:
                print("End of path reached!")
                end_reached = True
                break

            if lat_error is None and yaw_error is None:
                print("Errors too large")
                break

            calculation_results.append([converted_coords, current_heading, lat_error, yaw_error])

            input_state = np.array([lat_error / 2.5, yaw_error / np.pi])

            dqn_action = path_controller.act(np.reshape(input_state, [1, 2]))

            dqn_results.append([input_state, dqn_action])

            hardware_controller.set_steering_angle(dqn_action)

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("Manual stop...")
        hardware_controller.stop_control()

    finally:
        print("Stopping control")
        hardware_controller.stop_control()

    print("Saving data...")

    with open(f"TihanResults/{RUN_NAME}_gps.txt", "w+") as file:
        for point in gps_results:
            file.write(str(point) + "\n")

    with open(f"TihanResults/{RUN_NAME}_hw.txt", "w+") as file:
        for point in hardware_results:
            file.write(str(point) + "\n")

    with open(f"TihanResults/{RUN_NAME}_calc.txt", "w+") as file:
        for point in calculation_results:
            file.write(str(point) + "\n")

    with open(f"TihanResults/{RUN_NAME}_dqn.txt", "w+") as file:
        for point in dqn_results:
            file.write(str(point) + "\n")

    print("Data saved!")
