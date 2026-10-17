from pymavlink import mavutil  
from functions.func import *
from time import sleep

def take_picture(): # You can write picture taking logic here
    print("Picture taken")
    sleep(2)  # Simulate time taken to take a picture

def snake(azimuth, length, width, step, altitude):
    set_home()
    take_picture()
    for i in range(0, width//step):
        for j in range(0, length//step):
            az = azimuth
            if i % 2 == 1:
                az = az + 180
            go_to(az, step, altitude)
            take_picture()
        if i % 2 == 1:
            az = azimuth - 90
        else:
            az = azimuth + 90
        go_to(az, step, altitude)
        take_picture()
    go_home()

connection = mavutil.mavlink_connection(
    "udpin:127.0.0.1:14550",  # listening on UDP port 14550
    source_system=245,  # set the system ID for this connection
)

print("Waiting for heartbeat...") 
heartbeat = connection.wait_heartbeat(timeout=15)

if heartbeat is None:  
    connection.close()  
    raise SystemExit("No heartbeat received.")

print("Connected!")  
print("Flight mode:", connection.flightmode)  

init_conn(connection) # Ia zaebalsia pihat ego v kazhduiu funkciu, luchshe odin raz propisat
guided()
arm()
takeoff(10) # altitude
# go_to(90, 5, 10) # azimuth, distance, altitude
snake(90, 20, 20, 5, 15) #azimuth, length, width, step, altitude
land()
connection.close()  




 


