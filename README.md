# Drone Flight Test

A Python project for testing drone flight control, and automated area photography.

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

- **`math_func.py`** — Functions for geographic coordinate calculations and transformations, including calculating new coordinates based on direction and distance.
- **`func.py`** — Drone control functions, including arming and disarming, flight mode selection, takeoff, and navigation to a target point.
- **`camera.py`** — Reserved for future implementation of camera control and interaction.
- **`track.py`** — Functions for area coverage and automated photography, including flight patterns for surveying a specified area.

## Current Tasks

- [ ] **Implement precomputed lawnmower flight paths.** Calculate all waypoints before takeoff to minimize cumulative GPS coordinate errors during flight.
- [ ] **Introduce a local coordinate system.** Replace the current azimuth-and-distance navigation model with a custom coordinate system for easier waypoint planning and path generation.
- [ ] **Maintain camera orientation during lawnmower flights.** Keep the drone's heading consistent throughout the flight pattern to avoid unwanted changes in camera orientation.
- [ ] **Implement camera support in Gazebo** Integrate the simulated camera stream into the Python code to enable experiments with aerial mapping.
- [ ] **Develop a 2D mapping script** Create a Python script to process captured images and generate a 2D map.

## Development Goals

The main goal is to develop a modular system for automated drone flights and area photography, with reliable waypoint planning, flexible flight path generation, and consistent camera orientation.

The project is currently under development, and its structure and functionality may change as new features are implemented.
