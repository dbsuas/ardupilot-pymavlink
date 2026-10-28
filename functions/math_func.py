import math
import functions.func as fun

R = 6378137.0 #Earth radius

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

def enu_to_geo(E, N):
    phi = math.radians(fun.enu_home_lat)
    omega = math.radians(fun.enu_home_lon)

    a = R * math.cos(phi) * math.cos(omega) - E * math.sin(omega) - N * math.sin(phi) * math.cos(omega)
    b = R * math.cos(phi) * math.sin(omega) + E * math.cos(omega) - N * math.sin(phi) * math.sin(omega)
    c = R * math.sin(phi) + N * math.cos(phi)

    k = 2.0 * (math.cos(phi)*(a * math.cos(omega) + b * math.sin(omega)) + c * math.sin(phi))
    m = a*a + b*b + c*c - R*R

    t = 0.5 * (math.sqrt(k*k - 4.0 * m) - k)

    x = t * math.cos(phi) * math.cos(omega) + a
    y = t * math.cos(phi) * math.sin(omega) + b
    z = t * math.sin(phi) + c
    
    lat = math.asin(z/R)
    lon = math.atan2(y, x)

    return math.degrees(lat), math.degrees(lon)

