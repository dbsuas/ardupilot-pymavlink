from pymavlink import mavutil
from functions.camera import * 
from functions.func import *
from time import sleep

azimuth = 0  # Set the initial azimuth angle for the snake pattern

connection = mavutil.mavlink_connection(  # Create the MAVLink connection
    "udpin:127.0.0.1:14550",  # Receive MAVProxy messages on this port
    source_system=245,  # Assign an ID to our script
)

print("Waiting for heartbeat...")  # Display the connection status
heartbeat = connection.wait_heartbeat(timeout=15)  # Wait for an autopilot heartbeat

if heartbeat is None:  # Stop if no heartbeat arrives
    connection.close()  # Release the connection
    raise SystemExit("No heartbeat received.")  # Explain why execution stopped

init_conn(connection)  # Pass the established connection to our flight functions
print("Connected!")  # Confirm successful initialization

init_conn(connection) # connect to the drone
start_camera_stream() # start the camera stream
set_camera_angle(connection, pitch=-62, yaw=0) # set the camera angle to look straight down
guided()
arm()
takeoff(50) # altitude
sleep(5) # wait for the drone to stabilize
for i in range(4): # fly in a square pattern
    take_geotagged_picture(connection) # take a picture
    go_to(azimuth, 25, 50)
    sleep(5) # go forward 5 meters
    azimuth += 90 # turn 90 degrees
land() # land the drone
stop_camera_stream() # stop the camera stream