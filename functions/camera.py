from time import sleep  # Pause execution for a few seconds
import subprocess  # Run an external program from Python
from pathlib import Path  # Build file and folder paths
from datetime import datetime, timezone # Generate a unique image filename, Use UTC time for GPS metadata
from pymavlink import mavutil  # Communicate with the drone via MAVLink
import csv  # Save GPS coordinates to a CSV file
import piexif  # Write GPS metadata into JPEG images
from functions.math_func import *  # Use math functions for GPS calculations

def start_camera_stream():

    # Define the Gazebo camera streaming topic
    camera_topic = (
        "/world/iris_runway/model/iris_with_gimbal/model/"
        "gimbal/link/pitch_link/sensor/camera/image/enable_streaming"
    )

    # Prepare the command to enable camera streaming
    command = [
        "gz", "topic",  # Gazebo topic utility
        "-t", camera_topic,  # Select the camera topic
        "-m", "gz.msgs.Boolean",  # Set the message type
        "-p", "data: 1"  # Enable streaming
    ]

    # Send the command to Gazebo
    subprocess.run(command, check=True, timeout=10)

    # Allow the video stream to initialize
    sleep(2)

    # Confirm that the command was executed
    print("Camera streaming enabled!")

def stop_camera_stream():

    camera_topic = (
        "/world/iris_runway/model/iris_with_gimbal/model/"
        "gimbal/link/pitch_link/sensor/camera/image/enable_streaming"
    )
    # Disable camera streaming in Gazebo
    command = [
        "gz", "topic",
        "-t", camera_topic,
        "-m", "gz.msgs.Boolean",
        "-p", "data: false"
    ]

    subprocess.run(command, check=True, timeout=10)

    print("Camera streaming disabled!")


def set_camera_angle(connection, pitch=-62, yaw=0): # -62 degrees was found to be the closest to the NADIR (straight down) view of the camera in Gazebo

    # Send the gimbal angle command to ArduPilot
    connection.mav.command_long_send(
        connection.target_system,  # Target system ID
        connection.target_component,  # Target component ID
        mavutil.mavlink.MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,  # Gimbal control command
        0,  # Confirmation
        pitch,  # Camera pitch in degrees
        yaw,  # Camera yaw in degrees
        float("nan"),  # Ignore pitch rate
        float("nan"),  # Ignore yaw rate
        0,  # Gimbal manager flags
        0,  # Primary gimbal ID
        0  # Unused parameter
    )

    print("Camera angle command sent:", pitch, yaw)


def take_picture():  # Receive and save one camera frame (also used in geo-tagging)
    folder = Path(__file__).resolve().parent.parent / "captures"  # Use the repository's captures folder
    folder.mkdir(parents=True, exist_ok=True)  # Create the folder if needed

    filename = datetime.now().strftime("frame_%Y%m%d_%H%M%S_%f.jpg")  # Include date, time, and microseconds
    picture_path = folder / filename  # Build the full output path

    command = [  # Describe the GStreamer command as separate arguments
        "gst-launch-1.0", "-q",  # Start GStreamer with minimal normal output
        "udpsrc", "port=5600",  # Receive camera packets on UDP port 5600
        "caps=application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264",  # Describe the incoming stream
        "!", "rtph264depay",  # Extract H.264 data from RTP packets
        "!", "avdec_h264",  # Decode the video into image frames
        "!", "videoconvert",  # Convert frames to a compatible pixel format
        "!", "jpegenc", "snapshot=true",  # Encode one frame as JPEG, then finish
        "!", "filesink", f"location={picture_path}",  # Write the JPEG to disk
    ]  # Finish the argument list

    subprocess.run(command, check=True, timeout=20)  # Wait for completion; fail on an error or timeout
    print("Picture saved:", picture_path)  # Display the saved file location
    return picture_path  # Return the path for later processing

def take_geotagged_picture(connection):

    # Get the current drone position from MAVLink
    position = connection.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=True,
        timeout=5
    )

    # Stop if no position data is available
    if position is None:
        raise RuntimeError("No GPS position received")

    # Read any newer position messages already available
    while True:  # <--- potential hazard: infinite loop if no newer messages are available !!!!!!!!!!!!!!
        newer = connection.recv_match(type="GLOBAL_POSITION_INT", blocking=False)

        # Stop when no more position messages are available
        if newer is None:
            break

        # Keep the latest available position
        position = newer

    # Convert latitude and longitude to degrees
    lat = position.lat / 1e7
    lon = position.lon / 1e7

    # Convert altitude above mean sea level to meters
    altitude = position.alt / 1000

    # Record the approximate capture time in UTC
    capture_time = datetime.now(timezone.utc)

    # Take a picture using our existing camera function
    picture_path = take_picture()

    # Prepare EXIF metadata
    exif_data = {
        "0th": {},
        "Exif": {},
        "GPS": {},
        "1st": {},
        "thumbnail": None
    }

    # Set latitude direction
    exif_data["GPS"][piexif.GPSIFD.GPSLatitudeRef] = b"N" if lat >= 0 else b"S"

    # Store latitude
    exif_data["GPS"][piexif.GPSIFD.GPSLatitude] = decimal_to_dms(lat)

    # Set longitude direction
    exif_data["GPS"][piexif.GPSIFD.GPSLongitudeRef] = b"E" if lon >= 0 else b"W"

    # Store longitude
    exif_data["GPS"][piexif.GPSIFD.GPSLongitude] = decimal_to_dms(lon)

    # Set altitude reference: above or below sea level
    exif_data["GPS"][piexif.GPSIFD.GPSAltitudeRef] = 0 if altitude >= 0 else 1

    # Store altitude in meters
    exif_data["GPS"][piexif.GPSIFD.GPSAltitude] = (round(abs(altitude) * 1000), 1000)

    # Store GPS date in UTC
    exif_data["GPS"][piexif.GPSIFD.GPSDateStamp] = capture_time.strftime("%Y:%m:%d").encode()

    # Store GPS time in UTC
    exif_data["GPS"][piexif.GPSIFD.GPSTimeStamp] = (
        (capture_time.hour, 1),
        (capture_time.minute, 1),
        (capture_time.second * 1000000 + capture_time.microsecond, 1000000)
    )

    # Store approximate image capture date
    exif_data["Exif"][piexif.ExifIFD.DateTimeOriginal] = capture_time.strftime("%Y:%m:%d %H:%M:%S").encode()

    # Write EXIF GPS data directly into the JPEG
    piexif.insert(piexif.dump(exif_data), str(picture_path))

    # Set the path for the GPS coordinates file
    csv_path = picture_path.parent / "image_geolocation.csv"

    # Add the image coordinates to the CSV file
    with open(csv_path, "a", newline="") as file:
        writer = csv.writer(file)
        writer.writerow([picture_path.name, lat, lon, altitude])

    # Display the saved image information
    print("Geotagged picture saved:", picture_path)
    print("Latitude:", lat)
    print("Longitude:", lon)
    print("Altitude MSL:", altitude, "m")

    # Return the saved picture path
    return picture_path

def process_picture():
    sleep(1)
    print("Picture processed")

def decimal_to_dms(value):

    # Convert coordinate to a positive number
    value = abs(value)

    # Extract whole degrees
    degrees = int(value)

    # Calculate minutes
    total_minutes = (value - degrees) * 60
    minutes = int(total_minutes)

    # Calculate seconds
    seconds = (total_minutes - minutes) * 60

    # Return degrees, minutes, and seconds in EXIF format
    return ((degrees, 1), (minutes, 1), (round(seconds * 1000000), 1000000))