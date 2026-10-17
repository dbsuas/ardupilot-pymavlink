import math

R = 6371000.0 # Earth radius

def distance_between(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(math.radians, (lat1, lon1, lat2, lon2))

    dlat = lat2 - lat1
    dlon = lon2 - lon1

    a = (math.sin(dlat / 2) ** 2
        + math.cos(lat1) * math.cos(lat2)
        * math.sin(dlon / 2) ** 2)

    a = max(0.0, min(1.0, a))
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c

def shift_coordinates(lat, lon, azimuth, distance):
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    az = math.radians(azimuth)

    delta = distance / R
    lat2 = math.asin(
        math.sin(lat1) * math.cos(delta)
        + math.cos(lat1) * math.sin(delta) * math.cos(az))
    lon2 = lon1 + math.atan2(
        math.sin(az) * math.sin(delta) * math.cos(lat1),
        math.cos(delta) - math.sin(lat1) * math.sin(lat2))
    new_lat = math.degrees(lat2)
    new_lon = (math.degrees(lon2) + 540) % 360 - 180

    return new_lat, new_lon
