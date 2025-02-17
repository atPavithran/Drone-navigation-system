# Drone Navigation System using HoverAviary

## Overview
This project is a **drone waypoint navigation system** built using the **HoverAviary** environment from `gym-pybullet-drones`. The system enables a drone to navigate through a sequence of predefined waypoints, visually marking each waypoint and updating its status upon reaching it. The drone's control mechanism is implemented using **DSLPIDControl**.

This project was developed for **VOLTAICS'25 Hackathon** conducted by **Havoltz Club**, where it won **First Place**. The problem statement was to **design a drone simulation that is able to navigate through an array of waypoints through a function call**.

## Features
- **Waypoint visualization:** Displays waypoints as red spheres in the PyBullet environment.
- **Navigation control:** Uses **DSLPIDControl** to guide the drone to each waypoint.
- **Real-time status updates:** Changes waypoint color to green when reached.
- **Hovering mechanism:** Ensures the drone hovers at each waypoint for a minimum time.
- **Simulation parameters:** Configurable physics engine, control frequency, and GUI.

## Demo
![Image](https://github.com/user-attachments/assets/b27eefd9-322d-4d16-957d-b739e23c9215)

## Installation
Clone this repository and navigate to the project directory:

```bash
git clone https://github.com/atPavithran/Drone-navigation-system.git
cd Drone-navigation-system
pip install -r requirements.txt
```

## Usage
Run the script with default waypoints:

```bash
python solution.py
```

To specify custom waypoints, modify the `waypoints` list in the script:

```python
waypoints = [(0, 0, 1), (2, 2, 2), (3, 3, 2), (2, 2, 1)]
```

## Code Explanation
### `visualize_waypoints(env, waypoints, sphere_size=0.13)`
Adds visual sphere markers and numerical labels to waypoints in the PyBullet simulation.

### `update_waypoint_color(waypoint_id, env)`
Updates the waypoint color to green when reached by the drone.

### `waypoint_navigation(waypoints, ...)`
Main function that initializes the environment, sets up the drone controllers, and simulates the navigation process.

## Configuration
Modify parameters such as:

- **GUI Mode:** Set `gui=False` to run without visualization.
- **Simulation Duration:** Adjust `duration_sec` for longer or shorter runs.
- **Control Frequency:** Tune `control_freq_hz` for smoother navigation.

## Acknowledgment
This project was developed as part of **VOLTAICS'25 Hackathon** conducted by **Havoltz Club**, where it secured **First Place**.

## License
This project is open-source and available under the MIT License.

