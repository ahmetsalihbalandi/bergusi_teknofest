import cv2
import numpy as np
import time
import pickle
import os.path
import struct
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import argparse

def connectMyCopter(connection_string):
    baud_rate = 57600
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle

def arm_and_takeoff(aTargetAltitude, vehicle):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
        
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def goto_position_target_relative_ned(velocity_x, velocity_y, velocity_z, vehicle):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       
        0, 0,    
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, 
        0b0000111111000111,  
        0, 0, 0,  
        velocity_x, velocity_y, velocity_z,  
        0, 0, 0,  
        0, 0)
    vehicle.send_mavlink(msg)

def LAND(vehicle):
    vehicle.mode = VehicleMode("LAND")
    print(" Mode: %s" % vehicle.mode.name) 

    while vehicle.mode.name != "LAND":
        time.sleep(1)
        print("Vehicle mode is: %s" % str(vehicle.mode.name))
        vehicle.mode = VehicleMode("LAND")

    print("Vehicle Mode is : LAND")

def condition_yaw(heading, relative, vehicle):
    if relative:
        is_relative = 1 
    else:
        is_relative = 0 
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0, 
        heading,    
        0,          
        1,          
        is_relative, 
        0, 0, 0)    
    vehicle.send_mavlink(msg)

def saveData(data, name):
    pickle_file = name + '.pkl'
    current_directory = os.path.dirname(os.path.abspath(__file__))
    pickle_file_path = os.path.join(current_directory, pickle_file)

    if os.path.isfile(pickle_file_path) and os.path.getsize(pickle_file_path) > 0:
        with open(pickle_file_path, 'rb') as f:
            location_data = pickle.load(f)
    else:
        location_data = []

    location_data.extend([data])

    with open(pickle_file_path, 'wb') as f:
        pickle.dump(location_data, f, protocol=2)
        f.close()

def readData(database):
    current_directory = os.path.dirname(os.path.abspath(__file__))
    pickle_file_path = os.path.join(current_directory, database)

    if os.path.isfile(pickle_file_path) and os.path.getsize(pickle_file_path) > 0:
        with open(pickle_file_path, 'rb') as f:
            try:
                location_data = pickle.load(f)
            except (EOFError, struct.error):
                location_data = []

        if len(location_data) > 0:
            return location_data[-1]
    else:
        return None

def colorProcess(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_red = np.array([161, 155, 84])
    upper_red = np.array([179, 255, 255])

    mask = cv2.inRange(hsv, lower_red, upper_red)
    _, mask = cv2.threshold(mask, 254, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300:
                perimeter = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.04 * perimeter, True)
                if len(approx) == 4:
                    x, y, w, h = cv2.boundingRect(cnt)
                    return (x, y, w, h)

    return None

def capture_image():
    # Placeholder for image capture function
    # Replace this with actual code to capture an image from the drone's camera
    return cv2.imread('path_to_test_image.jpg')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Drone connection string')
    parser.add_argument('--connect', type=str, required=True, help='Connection string for drone')
    args = parser.parse_args()

    if args.connect:
        # Connect to drone
        vehicle = connectMyCopter(args.connect)

        # Take off
        arm_and_takeoff(10, vehicle)
        time.sleep(3)

        while True:
            # Capture image from the drone's camera
            frame = capture_image()  # Replace this with the actual image capture code

            if frame is not None:
                color_counts = colorProcess(frame)
                if color_counts is not None:
                    x, y, w, h = color_counts
                    center_x = x + w / 2
                    center_y = y + h / 2

                    # Calculate the necessary movement to center over the red rectangle
                    error_x = center_x - frame.shape[1] / 2
                    error_y = center_y - frame.shape[0] / 2

                    # Apply corrections
                    if error_x > 20:
                        condition_yaw(vehicle.heading + 1, True, vehicle)
                    elif error_x < -20:
                        condition_yaw(vehicle.heading - 1, True, vehicle)
                    if error_y > 20:
                        goto_position_target_relative_ned(0, 0, 0.1, vehicle)
                    elif error_y < -20:
                        goto_position_target_relative_ned(0, 0, -0.1, vehicle)
                    else:
                        # Land if the drone is correctly positioned over the red rectangle
                        LAND(vehicle)
                        break
                time.sleep(1)
        vehicle.close()
