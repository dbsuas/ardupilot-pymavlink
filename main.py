from pymavlink import mavutil  
from functions.func import *
from functions.track import *

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

init_conn(connection) 
guided()
arm()
takeoff(10)
snake_mod(90, 20, 20, 5, 15) #azimuth, length, width, step, altitude
land()
connection.close()  




 


