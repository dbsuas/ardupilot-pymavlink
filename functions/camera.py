from time import sleep  # Pause execution for a few seconds
import subprocess  # Run an external program from Python
from pathlib import Path  # Build file and folder paths
from datetime import datetime  # Generate a unique image filename
from pymavlink import mavutil  # Communicate with the drone via MAVLink

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


def take_picture():  # Receive and save one camera frame
    folder = Path(__file__).resolve().parent.parent / "captures"  # Use the repository's captures folder
    folder.mkdir(parents=True, exist_ok=True)  # Create the folder if needed

    filename = datetime.now().strftime("frame_%Y%m%d_%H%M%S_%f.png")  # Include date, time, and microseconds
    picture_path = folder / filename  # Build the full output path

    command = [  # Describe the GStreamer command as separate arguments
        "gst-launch-1.0", "-q",  # Start GStreamer with minimal normal output
        "udpsrc", "port=5600",  # Receive camera packets on UDP port 5600
        "caps=application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264",  # Describe the incoming stream
        "!", "rtph264depay",  # Extract H.264 data from RTP packets
        "!", "avdec_h264",  # Decode the video into image frames
        "!", "videoconvert",  # Convert frames to a compatible pixel format
        "!", "pngenc", "snapshot=true",  # Encode one frame as PNG, then finish
        "!", "filesink", f"location={picture_path}",  # Write the PNG to disk
    ]  # Finish the argument list

    subprocess.run(command, check=True, timeout=20)  # Wait for completion; fail on an error or timeout
    print("Picture saved:", picture_path)  # Display the saved file location
    return picture_path  # Return the path for later processing

def process_picture():
    sleep(2)
    print("Picture processed")
