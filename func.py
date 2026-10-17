import math
from pymavlink import mavutil

connection = None

home_lat = 0
home_lon = 0
home_height = 0

def init_conn(conn):
    global connection
    connection = conn

def distance_between(lat1, lon1, lat2, lon2):
    R = 6371000.0  # Средний радиус Земли, м

    # Перевод координат в радианы
    lat1, lon1, lat2, lon2 = map(math.radians, (lat1, lon1, lat2, lon2))

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Формула гаверсинусов
    a = (math.sin(dlat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2)
        * math.sin(dlon / 2) ** 2)

    # Защита от погрешностей округления
    a = max(0.0, min(1.0, a))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c

def shift_coordinates(lat, lon, azimuth, distance):
    R = 6371000.0  # Средний радиус Земли, м

    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    az = math.radians(azimuth)

    # Угловое расстояние
    delta = distance / R
    # Новая широта
    lat2 = math.asin(
        math.sin(lat1) * math.cos(delta)
        + math.cos(lat1) * math.sin(delta) * math.cos(az))
    # Новая долгота
    lon2 = lon1 + math.atan2(
        math.sin(az) * math.sin(delta) * math.cos(lat1),
        math.cos(delta) - math.sin(lat1) * math.sin(lat2))
    # Возвращаем градусы
    new_lat = math.degrees(lat2)
    new_lon = (math.degrees(lon2) + 540) % 360 - 180

    return new_lat, new_lon

def guided():
    connection.set_mode (connection.mode_mapping()['GUIDED'])  # set the flight mode to GUIDED
    while True:
        message = connection.recv_match(type='HEARTBEAT', blocking=True, timeout=2)
        if message and message.custom_mode == connection.mode_mapping()['GUIDED']:
            print("GUIDED mode enabled")
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
            print("Target position reached!")
            break

def set_home():
    message = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    global home_lat, home_lon, home_height
    home_lat = message.lat
    home_lon = message.lon
    home_height = message.relative_alt / 1000
    print("Home position was set")

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
            print("Home position reached!")
            break

