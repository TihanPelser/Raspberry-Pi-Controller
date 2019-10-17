from gps_interface.ublox_interface import UBX
import sys

if __name__ == "__main__":
    LOG_FILE = sys.argv[1]

    gps = UBX()

    gps.start_reading()

    print("Waiting for gps to set up")
    while gps.lat == 0:
        continue

    points = []
    print("Press ENTER to start log a point")
    print("Enter <S> to stop measuring and save data")
    while True:
        user_input = input("Enter a command:\n")
        if user_input == "":
            point = [gps.lat, gps.long]
            points.append(point)
            print(f"Captured point {point}")
        elif user_input.upper() == "S":
            print("Stopping path measurement")
            break
        else:
            continue
    with open(f"paths/{LOG_FILE}.txt", "w+") as file:
        for point in points:
            file.write(f"{point[0]},{point[1]}\n")


