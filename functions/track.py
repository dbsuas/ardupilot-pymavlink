from functions.camera import *
from functions.func import *

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
