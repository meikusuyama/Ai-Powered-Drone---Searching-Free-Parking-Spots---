#!/usr/bin/env python3
import time
import argparse
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import cv2
import tempfile
import random
import os
from ultralytics import YOLO
import socket
import math
HOME = os.getcwd()
import numpy as np
from picamera2 import Picamera2
import torch
from IPython.display import clear_output, display, Image
import threading

# ----------------------------------------------------------------------------------------------------------
# Defining constants
model=YOLO('models/best.pt')
image_width = 640
image_height = 360
centered_x_threshold = image_width/10       #10 is the # of parking spots we anticipate to fit in the frame
centered_y_threshold = image_height/10
target_altitude = 1.0
visited_coords = []  # Store last 3 visited coordinates as (lat, lon) tuples for fly_to_if_unique_and_check_boxes
guided_enabled = False
laptop_IP = ""       #IP address of the device the location is sent to
laptop_port = 3000
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
no_free_box_count_threshold = 5
# ----------------------------------------------------------------------------------------------------------



latest_frame = None
frame_lock   = threading.Lock()

#-------------------------------------------------
#starts up the picam
def camera_worker():
    global latest_frame
    picam2 = Picamera2()
    video_config = picam2.create_video_configuration(
        main={'size': (image_width, image_height), 'format': 'RGB888'},
        buffer_count=3
    )
    picam2.configure(video_config)
    picam2.start()

    while True:
        frame = picam2.capture_array()
        with frame_lock:
            latest_frame = frame
        time.sleep(0.05)

#-------------------------------------------------------




# ----------------------------------------------------------------------------------------------------------
# Returns a frame from camera feed
def get_camera_frame():
    with frame_lock:
        return None if latest_frame is None else latest_frame.copy()

    frame = picam2.capture_array()
    return frame
# ----------------------------------------------------------------------------------------------------------




# ----------------------------------------------------------------------------------------------------------
# Returns an array containing all the boxes detected
def detect_slots(frame):
    fixed_width = 1300 
    aspect_ratio = frame.shape[0] / frame.shape[1]  # height / width

    new_height = int(fixed_width * aspect_ratio)
    frame = cv2.resize(frame, (fixed_width, new_height), interpolation=cv2.INTER_AREA)
    
    with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
        cv2.imwrite(tmp.name, frame)
        results = model.predict(source=tmp.name, conf=0.25, save=False, verbose=False)

    return results
# ----------------------------------------------------------------------------------------------------------




# ----------------------------------------------------------------------------------------------------------
# Arm and takeoff
def arm_and_takeoff(target_alt):
    print(f"Arming and taking off to {target_alt}m...", flush=True)
    while not vehicle.is_armable:
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)
    vehicle.simple_takeoff(target_alt)
    # wait until the vehicle reaches a safe height
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.1f}m", flush=True)
        if alt >= target_alt * 0.95:
            print("Reached target altitude", flush=True)
            break
        time.sleep(1)
# ----------------------------------------------------------------------------------------------------------




# ----------------------------------------------------------------------------------------------------------
# Draws a grid of the relative positions of the box relative to the drone
# err_x, err_y: pixel differences between drone and box
# grid_width: number of characters horizontally (e.g., 48 for 640 width at ~13px per char)
# grid_height: number of characters vertically (e.g., 27 for 360 height at ~13px per char)
# marker_spacing: pixels per character cell

def draw_position_grid(err_x, err_y, grid_width=48, grid_height=27, marker_spacing=20):
    center_x = grid_width // 2
    center_y = grid_height // 2
    grid = [['.' for _ in range(grid_width)] for _ in range(grid_height)]

    # Place drone at center
    drone_pos = (center_x, center_y)
    grid[drone_pos[1]][drone_pos[0]] = 'o'

    # Calculate box position relative to drone
    dx = int(err_x / marker_spacing)
    dy = int(err_y / marker_spacing)
    box_x = max(0, min(grid_width - 1, center_x + dx))
    box_y = max(0, min(grid_height - 1, center_y + dy))

    if (box_x, box_y) != drone_pos:
        grid[box_y][box_x] = 'B'

    # Print grid
    for row in grid:
        print(''.join(row))
# ----------------------------------------------------------------------------------------------------------






# ----------------------------------------------------------------------------------------------------------
# Sends velocity commands north, east, and down in m/s
def send_ned_velocity(north, east, down, duration):
    print("!!! In send velocity commands")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (ignored)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,  # only velocity
        0, 0, 0,            # position
        north, east, down,  # velocity
        0, 0, 0,            # accelerations
        0, 0)
    for _ in range(int(duration*10)):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)
# ----------------------------------------------------------------------------------------------------------






# Helper function specifically for calibrate_perimeter
# ----------------------------------------------------------------------------------------------------------
# Move a certain direction for significantly more time than send_ned_velocity
# For covering bigger distances that don't need precision 
# Movement is relative to NED frame
def move_relative_ned(north, east, down=0, duration=0.001):
    """Send velocity command in NED frame."""
    print("Sending Calibration Velocity")
    send_ned_velocity(north, east, down, duration)
    time.sleep(duration)
# ----------------------------------------------------------------------------------------------------------









# ----------------------------------------------------------------------------------------------------------
# Return to home using GPS
def return_to_home(home_location):
    """Return to home using GPS."""
    vehicle.simple_goto(home_location)
    time.sleep(10)


# ----------------------------------------------------------------------------------------------------------
    """
    Calibrates the parking lot perimeter by flying in N, S, E, W directions until no parking boxes are detected.

    Results = array of ALL detections found in a frame (if the model detects no identifiable objects, the length of results[] will be 0)
    result = a SINGLE detection in a frame

    Arguments:
    - vehicle: dronekit Vehicle object
    - get_camera_frame: function to get image from camera
    - detect_slots: function to run detection on a frame
    - step_size: distance (m) to move per step in each direction
    - max_steps: maximum number of steps to attempt in each direction
    - no_box_threshold: number of consecutive frames with no boxes before stopping

    Returns:
    - Dictionary with locations for maximums of 'N', 'S', 'E', 'W'
    """
def calibratePerimeter(vehicle, home_location, step_size = 0.0001, max_steps = 10, no_box_threshold = 3):
    perimeter = {}
    directions = {
        "N": (0, step_size),
        "S": (0, -step_size),
        "E": (step_size, 0),
        "W": (-step_size, 0)
    }

    for direction, (vx, vy) in directions.items():
        print(f"Calibrating {direction}...")
        steps_taken = 0
        no_box_count = 0
        max_distances = {}
        distance = 0

        while steps_taken < max_steps:
            print(f"Number of steps taken: {steps_taken}")
            print(f"No Box Count {no_box_count}")

            move_relative_ned(vy, vx, 0, 0.0001)
            frame = get_camera_frame()
            results = detect_slots(frame)
            
            if no_box_count >= no_box_threshold:
                print(f"Stopped at {steps_taken} steps in direction {direction}")
                break

            elif len(results[0].boxes) == 0:
                print("!!!No box detected")
                no_box_count += 1

            else:
                no_box_count = 0 
                for result in results:
                    print(f"Number of boxes detected: {len(result.boxes)}")
                    steps_taken += 1
                    distance += step_size
        print(f"Max Distance (steps) in {direction} direction = {distance}")
        print(f"Border location in {direction} direction = {vehicle.location.global_relative_frame}")
        max_distances[direction] = distance
        perimeter[direction] = vehicle.location.global_relative_frame
        #hover in place
        vehicle.simple_goto(vehicle.location.global_relative_frame)
        time.sleep(5)
        print(f"Returning to home...")
        return_to_home(home_location)
        time.sleep(5)  # wait for stabilization

    print(f"Perimeter calibration complete: {perimeter}")
    return perimeter
# ----------------------------------------------------------------------------------------------------------







# ----------------------------------------------------------------------------------------------------------
# Approximate distance (in meters) between two lat/lon points using the Pythagorean theorem. Assumes the Earth is flat over small distances.
def distance_meters(lat1, lon1, lat2, lon2):
    # Approximate length of 1 degree of latitude and longitude in meters
    meters_per_deg_lat = 111_320  # constant
    meters_per_deg_lon = 40075000 * math.cos(math.radians((lat1 + lat2) / 2)) / 360

    d_lat = (lat2 - lat1) * meters_per_deg_lat
    d_lon = (lon2 - lon1) * meters_per_deg_lon

    return math.sqrt(d_lat**2 + d_lon**2)
# ----------------------------------------------------------------------------------------------------------








# ----------------------------------------------------------------------------------------------------------
# When no free boxes are detected for three camera frames: 
# Selects a random GPS coordinate and travels to it. If it finds an open parking spot along the way, it will start centering above it.
# Latitude = N-S --> y
# Longitude = E-W --> x
'''
Args:
        home_location: dronekit.LocationGlobalRelative (lat, lon, alt)
        perimeter: dict with keys "N", "S", "E", "W" representing max location (lat,lon) away from home location

Returns:
        LocationGlobalRelative object for the new coordinate
'''
def random_coordinates_within_perimeter(home_location, parking_perimeter):
    # Randomly pick a location between the parking lot boundaries
    target_y = random.uniform(parking_perimeter['S'].lat, parking_perimeter['N'].lat)
    target_x = random.uniform(parking_perimeter['W'].lon, parking_perimeter['E'].lon)

    return LocationGlobalRelative(target_y, target_x, target_altitude)
# ----------------------------------------------------------------------------------------------------------








# ----------------------------------------------------------------------------------------------------------
# Check if location was visited within threshold distance.
def already_visited(target_lat, target_lon, threshold_m=0.5):
    for lat, lon in visited_coords:
        if distance_meters(lat, lon, target_lat, target_lon) < threshold_m:
            return True
    return False
# ----------------------------------------------------------------------------------------------------------







# ----------------------------------------------------------------------------------------------------------
# Fly to target GPS coordinate only if not previously visited and stop if a free box is detected along the way.
def fly_to_if_unique_and_check_boxes(vehicle, home_location, target_lat, target_lon, parking_perimeter):
    if already_visited(target_lat, target_lon):
        print("🚫 Skipping: Location already visited (within 0.5 meters)")
        return

    # Command drone to the new location
    target_location = LocationGlobalRelative(target_lat, target_lon, target_altitude)
    vehicle.simple_goto(target_location)
    print(f"Flying to {target_lat}, {target_lon}")

    #up∂ate the visited array
    if(len(visited_coords) >= 3):
        visited_coords.pop(0)
    visited_coords.append((target_lat, target_lon))

    # While flying, periodically check for free boxes
    while True:
        current_location = vehicle.location.global_relative_frame
        dist_to_target = distance_meters(current_location.lat, current_location.lon, target_lat, target_lon)
        print(f"📍 Distance to target: {dist_to_target:.2f}m")

        # Get current camera frame
        frame = get_camera_frame()
        results = detect_slots(frame)

        for result in results:
            for box in result.boxes:
                label = result.names[int(box.cls[0])]
                if label == "free":
                    print("✅ Free box detected! Stopping movement.")
                    vehicle.simple_goto(current_location)  # Stop and hover in place
                    return

        if dist_to_target < 1.0:  # Target reached without finding free box
            print("📍 Arrived at destination")
            #generate another target coordinate
            second_target_location = random_coordinates_within_perimeter(home_location, parking_perimeter)
            #recurive call until it starts detecting free boxe
            fly_to_if_unique_and_check_boxes(vehicle, home_location, second_target_location.lat, second_target_location.lon, perimeter)
            return
        time.sleep(1)  # Slow polling to reduce load  
# ----------------------------------------------------------------------------------------------------------





# ----------------------------------------------------------------------------------------------------------
# MAIN       

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='udp:127.0.0.1:14551')
    args = parser.parse_args()

    print(f"Connecting to vehicle on {args.connect}")
    vehicle = connect("udp:127.0.0.1:14551", wait_ready=True)

    vehicle.airspeed = 0.0001      # affects Guided, RTL, Auto, etc.
    vehicle.groundspeed = 0.0001   

    arm_and_takeoff(target_altitude)


    cam_thread = threading.Thread(target=camera_worker, daemon=True)
    cam_thread.start()

    # wait up to 5 seconds for the first frame
    timeout = time.time() + 5.0
    frame = None
    while time.time() < timeout:
        frame = get_camera_frame()
        if frame is not None:
            break
        time.sleep(0.1)

    if frame is None:
        print("❌ No camera frame received after 5s")
    else:
        print("✅ Camera frame received")



    home_location = vehicle.location.global_relative_frame
    home_location = LocationGlobalRelative(home_location.lat, home_location.lon, target_altitude)

    perimeter = calibratePerimeter(vehicle, home_location) #dictionary of max distance away from home in 4 primary directions in meters
    no_free_box_count = 0


    while True:
        print("!!! In Parking Slot Loop")   
        frame = get_camera_frame()          #frame  = one image

        if frame is None:
            print("Frame not received")
            continue

        print("Frame received")


        results = detect_slots(frame)       #results = object that contains info about the one frame
                                           

        for result in results:              
            x_min = None
            y_min = None
            min_dist = float('inf')

            print(f"Number of boxes: {len(result.boxes)}")   #len(results.boxes) = number of boxes detected in the one frame

            for box in result.boxes:        
                box_coordinates = box.xyxy[0].cpu().numpy()
                print("!!! Received Box Coordinates")
                cls_id = int(box.cls[0])
                label = result.names[cls_id]
                if label == "free":
                    print("!!! Free Label Found")
                    no_free_box_count = 0
                    x = (box_coordinates[0] + box_coordinates[2]) / 2
                    y = (box_coordinates[1] + box_coordinates[3]) / 2

                    x_diff = x - frame.shape[1] / 2
                    y_diff = y - frame.shape[0] / 2
                    distance = math.sqrt(x_diff**2 + y_diff**2)

                    if distance < min_dist:
                        x_min = x
                        y_min = y
                        min_dist = distance

            # Only send velocity if a valid free spot was found
            if x_min is not None and y_min is not None:
                err_x = frame.shape[1] / 2 - x_min
                err_y = frame.shape[0] / 2 - y_min

                draw_position_grid(err_x, err_y)

                if(abs(err_x) <= centered_x_threshold and abs(err_y) <= centered_y_threshold):
                    print("The drone is CENTERED")
                    vehicle.simple_goto(vehicle.location.global_relative_frame) # hover in place
                    # send the location to the device
                    loc = vehicle.location.global_relative_frame
                    msg = f"{loc.lat:.8f},{loc.lon:.8f},{loc.alt:.2f}"
                    udp_sock.sendto(msg.encode("utf-8"), (laptop_IP, laptop_port))
                    print(f"Sent location to laptop: {msg}")
                    #Go back home and land
                    vehicle.simple_goto(home_location)
                    #hover for a few seconds
                    time.sleep(5)
                    print("Landing...", flush=True)
                    vehicle.mode = VehicleMode("LAND")

                    #Close vehicle object before exiting script
                    time.sleep(5)
                    vehicle.close()
                    print("Done", flush=True)


                
                else:
                    print("-------------------------------------------------------------------------")
                    vx = err_x * 0.0001  # Adjusted gain
                    vy = -err_y * 0.0001

                    if vx > 0:
                        print("→ Drone move RIGHT (positive x error)")
                    elif vx < 0:
                        print("← Drone move LEFT (negative x error)")

                    if vy > 0:
                        print("↑ Drone move FORWARD/NORTH (positive y error)")
                    elif vy < 0:
                        print("↓ Drone move BACKWARD/SOUTH (negative y error)")
                    print("Centering Velocity Sent")
                    send_ned_velocity(vy, vx, 0, 0.0001)
            else:   #it doesn't detect any free box in the frame
                if (no_free_box_count < no_free_box_count_threshold):
                    no_free_box_count += 1
                    print(f"No Free Box Count = {no_free_box_count}")
                    break
                #if no free spot detected in the frame within the threshold, generate a random location within the perimeter and fly toward it
                print("No free spots. Generating new random coordinate")
                target_location = random_coordinates_within_perimeter(home_location, perimeter)
                fly_to_if_unique_and_check_boxes(vehicle, home_location, target_location.lat, target_location.lon, perimeter)
                break

                