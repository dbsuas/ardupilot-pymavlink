# Drone Flight Test

A Python project for testing drone flight control, coordinate calculations, and automated area photography.

## Project Structure

```text
.
├── fly_test/          # Test flight scripts
├── functions/         # Reusable functions
│   ├── math_func.py   # Coordinate transformation and calculation functions
│   ├── func.py        # Drone control and flight operations
│   ├── camera.py      # Camera interaction (planned)
│   └── track.py       # Area coverage and photography functions
└── main.py            # Current test flight
```

## Modules

### `main.py`
The entry point for the current test flight. Used to run and test the active flight sequence.

### `fly_test/`
Contains scripts for experimental and test flights.

### `functions/`

#### `math_func.py`
Functions for geographic coordinate calculations and coordinate transformations, including calculating new positions based on direction and distance.

- `distance_between(lat1, lon1, lat2, lon2)` — calculates the distance between two geographic points.
- `shift_coordinates(lat, lon, azimuth, distance)` — calculates new geographic coordinates by applying a displacement with a specified azimuth and distance.
- `enu_to_geo(East, North)` — converts coordinates from the local ENU (East-North-Up) coordinate system to geographic coordinates.

#### `func.py`
Drone control functions, including connection initialization, flight mode selection, takeoff, landing, and navigation to target points.

- `init_conn(conn)` — passes the connection object to the module's drone control functions.
- `guided()` — switches the drone to Guided flight mode.
- `land()` — switches the drone to Land mode.
- `arm()` — arms the drone.
- `takeoff(altitude)` — takes off to the specified altitude.
- `go_to(azimuth, distance, altitude)` — navigates the drone to a target point specified by azimuth, distance, and altitude.
- `go_to_enu(East, North, altitude)` — navigates the drone to a target point specified in the local ENU coordinate system. The ENU home position must be initialized before use.
- `set_home()` — sets the home position used for return-to-home operations.
- `set_enu_home()` — initializes the origin of the local ENU coordinate system.
- `go_home()` — returns the drone to the configured home position.
- `yaw_azimuth(azimuth)` — rotates the drone in yaw to a specified azimuth.

#### `camera.py`
Functions for camera control and interaction. Implementations will be added in the future.

- `take_picture()` — captures a photograph. Implementation planned for future development.
- `process_picture()` — processes a captured photograph. Implementation planned for future development.

#### `track.py`
Functions for area coverage planning and automated aerial photography, including flight path generation for surveying a specified area.

- `snake(azimuth, length, width, step, altitude)` — the legacy, less accurate implementation of the lawnmower flight pattern.
- `snake_mod(azimuth, length, width, step, altitude)` — the updated lawnmower flight pattern implementation using the local ENU coordinate system.

## Current Tasks

- [+] **Implement precomputed lawnmower flight paths.** Calculate all waypoints before takeoff to minimize cumulative GPS coordinate errors during flight.
- [+] **Introduce a local coordinate system.** Replace the current azimuth-and-distance navigation model with a custom coordinate system for easier waypoint planning and path generation.
- [+] **Maintain camera orientation during lawnmower flights.** Keep the drone's heading consistent throughout the flight pattern to avoid unwanted changes in camera orientation.
- [+] **Implement camera support in Gazebo.** Integrate the simulated camera stream into the Python code to enable experiments with aerial mapping.
- [ ] **Add geotagging** Retrieve the geographic coordinates of each captured image for use in the mapping process.
- [ ] **Develop a 2D mapping script.** Create a Python script to process captured images and generate a 2D map.
## Development Goals

The main goal is to develop a modular system for automated drone flights and area photography, with reliable waypoint planning, flexible flight path generation, and consistent camera orientation.

The project is currently under development, and its structure and functionality may change as new features are implemented.
