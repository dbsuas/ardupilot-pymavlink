from pymavlink import mavutil  # Import MAVLink communication utilities
import math  # Import mathematical functions


# Calculate the distance between two GPS coordinates in meters
def distance_between(lat1, lon1, lat2, lon2):  # Calculate the great-circle distance between two GPS points.
    """
    lat1, lon1 — coordinates of the first point in degrees
    lat2, lon2 — coordinates of the second point in degrees

    Returns the distance in meters.
    """

    R = 6371000.0  # Define the Earth's average radius in meters

    # Convert GPS coordinates from degrees to radians
    lat1, lon1, lat2, lon2 = map(  # Convert all four coordinate values to radians.
        math.radians, (lat1, lon1, lat2, lon2)  # Apply the radians conversion to each coordinate.
    )  # Complete the calculation.

    dlat = lat2 - lat1  # Calculate the latitude difference
    dlon = lon2 - lon1  # Calculate the longitude difference

    # Calculate the Haversine formula
    a = (  # Combine the latitude and longitude terms of the Haversine formula.
        math.sin(dlat / 2) ** 2  # Calculate the latitude component
        + math.cos(lat1) * math.cos(lat2)  # Apply latitude correction
        * math.sin(dlon / 2) ** 2  # Calculate the longitude component
    )  # Complete the calculation.

    a = max(0.0, min(1.0, a))  # Prevent floating-point rounding errors

    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))  # Calculate angular distance

    return R * c  # Return the distance in meters


# Calculate new GPS coordinates using direction and distance
def shift_coordinates(lat, lon, azimuth, distance):  # Calculate the destination point for a bearing and distance.
    """
    lat, lon  — original coordinates in degrees
    azimuth   — azimuth in degrees (0° — north, 90° — east)
    distance  — distance in meters

    Returns (new_lat, new_lon) in degrees.
    """

    R = 6371000.0  # Define the Earth's average radius in meters

    # Convert input coordinates and azimuth to radians
    lat1 = math.radians(lat)  # Convert latitude to radians
    lon1 = math.radians(lon)  # Convert longitude to radians
    az = math.radians(azimuth)  # Convert azimuth to radians

    delta = distance / R  # Calculate angular distance

    # Calculate the destination latitude
    lat2 = math.asin(  # Calculate the destination latitude in radians.
        math.sin(lat1) * math.cos(delta)  # Calculate the initial latitude component
        + math.cos(lat1) * math.sin(delta) * math.cos(az)  # Apply distance and direction
    )  # Complete the calculation.

    # Calculate the destination longitude
    lon2 = lon1 + math.atan2(  # Add the angular longitude offset to the initial longitude.
        math.sin(az) * math.sin(delta) * math.cos(lat1),  # Calculate longitude numerator
        math.cos(delta) - math.sin(lat1) * math.sin(lat2)  # Calculate longitude denominator
    )  # Complete the calculation.

    # Convert calculated coordinates back to degrees
    new_lat = math.degrees(lat2)  # Convert latitude to degrees
    new_lon = (math.degrees(lon2) + 540) % 360 - 180  # Normalize longitude

    return new_lat, new_lon  # Return destination GPS coordinates


# --------------------------------------------------
# MAVLINK CONNECTION
# --------------------------------------------------

# Establish a MAVLink connection with ArduPilot
connection = mavutil.mavlink_connection(  # Create the MAVLink connection with the parameters below.
    "udpin:127.0.0.1:14550",  # Listen for incoming UDP packets on port 14550
    source_system=245,  # Assign system ID 245 to the Python application
)  # Complete the function call.

print("Waiting for heartbeat...")  # Display connection status

# Wait up to 15 seconds for an autopilot heartbeat
heartbeat = connection.wait_heartbeat(timeout=15)  # Wait up to 15 seconds for a heartbeat message.

# Check whether the heartbeat was received
if heartbeat is None:  # Handle the absence of an autopilot heartbeat.
    connection.close()  # Close the MAVLink connection
    raise SystemExit("No heartbeat received.")  # Exit if connection fails

# Display connection information
print("Connected!")  # Confirm successful connection
print("System ID:", connection.target_system)  # Display autopilot system ID
print("Component ID:", connection.target_component)  # Display component ID
print("Flight mode:", connection.flightmode)  # Display current flight mode

# Check whether the drone is armed
if connection.motors_armed():  # Check the current motor arming status.
    print("Drone is ARMED")  # Display armed status
else:  # Handle the disarmed state.
    print("Drone is DISARMED")  # Display disarmed status


# --------------------------------------------------
# FLIGHT MODE AND ARMING
# --------------------------------------------------

# Request GUIDED flight mode
connection.set_mode(connection.mode_mapping()['GUIDED'])  # Request the autopilot mode mapped to GUIDED.

# Wait until the autopilot reports GUIDED mode
while True:  # Keep receiving telemetry until the exit condition is met.
    msg = connection.recv_match(  # Read the requested MAVLink telemetry message.
        type='HEARTBEAT',  # Receive heartbeat messages
        blocking=True,  # Wait for incoming messages
        timeout=2  # Wait up to 2 seconds per attempt
    )  # Complete the function call.

    # Verify the current flight mode
    if msg and msg.custom_mode == connection.mode_mapping()['GUIDED']:  # Check that a received heartbeat reports GUIDED mode.
        print("GUIDED mode enabled")  # Confirm GUIDED mode
        break  # Exit the mode verification loop

connection.arducopter_arm()  # Send the motor arming command

connection.motors_armed_wait()  # Wait until motors are armed

print("Motors armed!")  # Confirm successful arming


# --------------------------------------------------
# AUTOMATIC TAKEOFF
# --------------------------------------------------

altitude = 10  # Set target takeoff altitude to 10 meters

# Send the MAVLink takeoff command
connection.mav.command_long_send(  # Send the takeoff command with its parameters.
    connection.target_system,  # Target autopilot system ID
    connection.target_component,  # Target autopilot component ID
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # MAVLink takeoff command
    0,  # Command confirmation
    0, 0, 0, 0, 0, 0,  # Unused command parameters
    altitude  # Target takeoff altitude in meters
)  # Complete the function call.

# Monitor altitude during takeoff
while True:  # Keep receiving telemetry until the exit condition is met.
    msg = connection.recv_match(  # Read the requested MAVLink telemetry message.
        type='GLOBAL_POSITION_INT',  # Receive global position telemetry
        blocking=True,  # Wait for the next position message
        timeout=2  # Wait up to 2 seconds per attempt
    )  # Complete the function call.

    # Skip the iteration if no message was received
    if msg is None:  # Handle a telemetry receive timeout.
        continue  # Skip this iteration and wait for another message.

    current_alt = msg.relative_alt / 1000  # Convert relative altitude from mm to m

    print("Altitude:", round(current_alt, 2), "m")  # Display current altitude

    # Check whether the target altitude has been reached
    if current_alt >= altitude - 0.5:  # Accept an altitude within 0.5 meters below the target or higher.
        print("Target altitude reached!")  # Confirm target altitude
        break  # Exit the altitude monitoring loop


# --------------------------------------------------
# GPS POSITION AND TARGET CALCULATION
# --------------------------------------------------

# Read the current GPS position
message = connection.recv_match(  # Wait for the current global position message.
    type='GLOBAL_POSITION_INT',  # Receive global position telemetry
    blocking=True  # Wait until a position message is available
)  # Complete the function call.

# Extract GPS coordinates and altitude
target_lat = message.lat / 1e7  # Convert latitude to degrees
target_lon = message.lon / 1e7  # Convert longitude to degrees
target_alt = message.alt / 1000.0  # Convert MSL altitude from mm to m

# Display the current GPS position
print(  # Print the current position values.
    "Position (lat, lon, alt):",  # Label the latitude, longitude, and altitude values.
    target_lat, target_lon, target_alt  # Supply the current coordinates and MSL altitude.
)  # Complete the function call.

# Calculate new coordinates 20 meters east of the current position
target_lat, target_lon = shift_coordinates(  # Replace the current coordinates with the destination coordinates.
    target_lat, target_lon, 90, 20  # Use a bearing of 90 degrees and a distance of 20 meters.
)  # Complete the function call.


# --------------------------------------------------
# GPS NAVIGATION
# --------------------------------------------------

# Send the target GPS position to ArduPilot
connection.mav.set_position_target_global_int_send(  # Send the global position target and control parameters.
    0,  # Timestamp
    connection.target_system,  # Target system ID
    connection.target_component,  # Target component ID
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Relative altitude coordinate frame
    3576,  # Type mask: use position only
    int(target_lat * 1e7),  # Target latitude
    int(target_lon * 1e7),  # Target longitude
    10,  # Target altitude in meters relative to Home
    0, 0, 0,  # Velocity parameters (ignored)
    0, 0, 0,  # Acceleration parameters (ignored)
    0, 0  # Yaw and yaw rate parameters (ignored)
)  # Complete the function call.

# Monitor the distance between the drone and its destination
while True:  # Keep receiving telemetry until the exit condition is met.
    msg = connection.recv_match(  # Read the requested MAVLink telemetry message.
        type='GLOBAL_POSITION_INT',  # Receive global position telemetry
        blocking=True,  # Wait for the next message
        timeout=2  # Wait up to 2 seconds per attempt
    )  # Complete the function call.

    # Skip the iteration if no message was received
    if msg is None:  # Handle a telemetry receive timeout.
        continue  # Skip this iteration and wait for another message.

    lat = msg.lat / 1e7  # Convert current latitude to degrees
    lon = msg.lon / 1e7  # Convert current longitude to degrees

    # Calculate the remaining distance to the target
    distance = distance_between(  # Calculate the distance from the current position to the target.
        lat, lon, target_lat, target_lon  # Supply the current and destination coordinates.
    )  # Complete the function call.

    print("Distance to target:", round(distance, 2), "m")  # Display remaining distance

    # Check whether the drone is within 2 meters of the destination
    if distance <= 2:  # Check whether the target is within 2 meters.
        print("Target position reached!")  # Confirm arrival at destination
        break  # Exit the navigation monitoring loop


# --------------------------------------------------
# CLOSE MAVLINK CONNECTION
# --------------------------------------------------

connection.close()  # Close the MAVLink connection
