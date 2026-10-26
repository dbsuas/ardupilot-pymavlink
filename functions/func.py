import math
from pymavlink import mavutil
from functions.math_func import *
from time import sleep

stab_delay = 2 # time to stabilize after moving in seconds

mask = (mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE | # zabey hui
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

connection = None 

home_lat = None
home_lon = None
home_height = None

enu_home_lat = None
enu_home_lon = None

def init_conn(conn):
    global connection
    connection = conn

def guided():
    connection.set_mode (connection.mode_mapping()['GUIDED'])  # set the flight mode to GUIDED
    while True:
        message = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if message and message.custom_mode == connection.mode_mapping()['GUIDED']:
            print("GUIDED mode enabled")
            break
def land():
    connection.set_mode (connection.mode_mapping()['LAND'])  # set the flight mode to LAND
    while True:
        message = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if message and message.custom_mode == connection.mode_mapping()['LAND']:
            print("land mode enabled")
            break

def arm():
    connection.arducopter_arm()
    connection.motors_armed_wait()
    print("Motors armed!")

def takeoff(altitude):
    connection.mav.command_long_send(connection.target_system, connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)
    while True:
        message = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
        if message is None:
            continue

        current_alt = message.relative_alt / 1000

        print("Altitude:", round(current_alt, 2), "m")

        if current_alt >= altitude - 0.5:
            sleep(stab_delay)
            print("Target altitude reached!")
            break

def go_to(azimuth, distance, altitude):
    message = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

    lat = message.lat / 1e7  # convert latitude from integer to degrees
    lon = message.lon / 1e7  # convert longitude from integer to degrees

    target_lat, target_lon = shift_coordinates(lat, lon, azimuth, distance)

    connection.mav.set_position_target_global_int_send(0, connection.target_system, connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 3576, int(target_lat * 1e7), int(target_lon * 1e7), altitude, 0, 0, 0, 0, 0, 0, 0, 0)
    while True:
        message = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)

        if message is None:
            continue

        lat = message.lat / 1e7  # convert latitude from integer to degrees
        lon = message.lon / 1e7  # convert longitude from integer to degrees
        distance = distance_between(lat, lon, target_lat, target_lon)

        print("Distance to target:", round(distance, 2), "m")

        if distance <= 2:
            sleep(stab_delay)
            print("Target position reached!")
            break

def go_to_enu(E, N, altitude, course):
    target_lat, target_lon = enu_to_geo(E, N)
    connection.mav.set_position_target_global_int_send(0, connection.target_system, connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask, int(target_lat * 1e7), int(target_lon * 1e7), altitude, 0, 0, 0, 0, 0, 0, math.radians(course), 0)
    while True:
        message = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)

        if message is None:
            continue

        lat = message.lat / 1e7  # convert latitude from integer to degrees
        lon = message.lon / 1e7  # convert longitude from integer to degrees
        distance = distance_between(lat, lon, target_lat, target_lon)

        print("Distance to target:", round(distance, 2), "m")

        if distance <= 2:
            sleep(stab_delay)
            print("Target position reached!")
            break

def set_home():
    message = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    global home_lat, home_lon, home_height
    home_lat = message.lat
    home_lon = message.lon
    home_height = message.relative_alt / 1000
    print("Home position was set")

def set_enu_home():
    message = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    global enu_home_lat, enu_home_lon
    enu_home_lat = message.lat / 1e7
    enu_home_lon = message.lon / 1e7
    print("ENU home position was set")

def go_home():
    connection.mav.set_position_target_global_int_send(0, connection.target_system, connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 3576, home_lat, home_lon, home_height, 0, 0, 0, 0, 0, 0, 0, 0)
    while True:
        message = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)

        if message is None:
            continue

        lat = message.lat / 1e7  # convert latitude from integer to degrees
        lon = message.lon / 1e7  # convert longitude from integer to degrees
        distance = distance_between(lat, lon, home_lat / 1e7, home_lon / 1e7)

        print("Distance to home:", round(distance, 2), "m")

        if distance <= 2:
            sleep(stab_delay)
            print("Home position reached!")
            break
def yaw_azimuth(azimuth):
    message = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

    lat = message.lat 
    lon = message.lon 
    height = round(message.relative_alt / 1000)
    connection.mav.set_position_target_global_int_send(0, connection.target_system, connection.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask, lat, lon, height, 0, 0, 0, 0, 0, 0, math.radians(azimuth), 0)
    while True:
        message = connection.recv_match(type='ATTITUDE', blocking=True, timeout=2)

        if message is None:
            continue

        yaw = math.degrees(message.yaw) % 360

        print("Set yaw: ", azimuth, " cur: ", yaw, " delta: ", math.fabs(yaw-azimuth))

        if math.fabs(yaw - azimuth) <= 5:
            sleep(stab_delay)
            print("Yaw azimuth reached!")
            break

