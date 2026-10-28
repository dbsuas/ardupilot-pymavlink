from functions.camera import *
from functions.func import *
import math

def snake(azimuth, length, width, step, altitude):
    set_home()
    take_picture()
    process_picture()
    for i in range(0, width//step):
        for j in range(0, length//step):
            az = azimuth
            if i % 2 == 1:
                az = az + 180
            go_to(az, step, altitude)
            take_picture()
            process_picture()
        go_to(azimuth+90, step, altitude)
        take_picture()
        process_picture()
    go_home()

def snake_mod(azimuth, length, width, step, altitude):
    set_home()
    set_enu_home()
    yaw_azimuth(azimuth)
    E = 0
    N = 0 
    take_picture()
    process_picture()
    a = length//step
    b = width//step
    for i in range(0, b):
        for j in range(0, a):
            az = azimuth
            if i % 2 == 1:
                az = az + 180
            E = E + step * math.sin(math.radians(az))
            N = N + step * math.cos(math.radians(az))
            go_to_enu(E, N, altitude, azimuth)
            take_picture()
            process_picture()
        E = E + step * math.sin(math.radians(azimuth+90))
        N = N + step * math.cos(math.radians(azimuth+90))
        if(i != b-1):
            go_to_enu(E, N, altitude, azimuth)
        take_picture()
        process_picture()
    go_home()
