import os
import time
import numpy as np
import pybullet as p

from gym_pybullet_drones.utils.enums import DroneModel, Physics
from gym_pybullet_drones.envs.HoverAviary import HoverAviary
from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool


def visualize_waypoints(env, waypoints, sphere_size=0.13):
    """Adds sphere markers and numerical labels to waypoints in the PyBullet environment."""
    waypoint_ids = []
    for i, wp in enumerate(waypoints):
        sphere_visual = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                                            radius=sphere_size,
                                            rgbaColor=[1, 0, 0, 1],  # Red color
                                            physicsClientId=env.getPyBulletClient())

        sphere_id = p.createMultiBody(baseVisualShapeIndex=sphere_visual,
                                      basePosition=wp,
                                      physicsClientId=env.getPyBulletClient())
        waypoint_ids.append(sphere_id)

        # Add numerical label above the sphere
        p.addUserDebugText(f"{i+1}", wp + np.array([0, 0, sphere_size + 0.1]), 
                           textSize=1.5, textColorRGB=[1, 1, 1], 
                           physicsClientId=env.getPyBulletClient())
    
    return waypoint_ids


def update_waypoint_color(waypoint_id, env):
    """Changes the waypoint's color to green once the drone reaches it."""
    p.changeVisualShape(waypoint_id, -1, rgbaColor=[0, 1, 0, 1], physicsClientId=env.getPyBulletClient())


def waypoint_navigation(waypoints, drone=DroneModel("cf2x"), num_drones=1, physics=Physics("pyb"),
                         gui=True, record_video=False, plot=True, user_debug_gui=False, obstacles=False,
                         simulation_freq_hz=240, control_freq_hz=48, duration_sec=20, colab=False):
    """Simulates drone navigation through a given set of waypoints using HoverAviary."""
    
    INIT_XYZS = np.array([[0, 0, 0.1] for _ in range(num_drones)])
    INIT_RPYS = np.array([[0, 0, 0] for _ in range(num_drones)])

    env = HoverAviary(drone_model=drone,
                      initial_xyzs=INIT_XYZS,
                      initial_rpys=INIT_RPYS,
                      physics=physics,
                      pyb_freq=simulation_freq_hz,
                      ctrl_freq=control_freq_hz,
                      gui=gui,
                      record=record_video)

    # Override methods dynamically
    env._computeObs = lambda: np.array([env._getDroneStateVector(0) for _ in range(num_drones)])
    env._preprocessAction = lambda action: np.array([np.clip(action[i, :], 0, env.MAX_RPM) for i in range(num_drones)])
    
    original_step = env.step
    def modified_step(action):
        """Overrides step function to preprocess actions before passing to the environment."""
        return original_step(env._preprocessAction(action))
    
    env.step = modified_step  # Override the step method

    PYB_CLIENT = env.getPyBulletClient()

    controllers = [DSLPIDControl(drone_model=drone) for _ in range(num_drones)]
    action = np.zeros((num_drones, 4))

    waypoint_ids = visualize_waypoints(env, waypoints)

    START = time.time()
    wp_counters = np.zeros(num_drones, dtype=int)
    num_wps = len(waypoints)

    waypoint_hover_time = np.zeros(num_drones)
    HOVER_THRESHOLD = 0.15  # Distance threshold for considering hovering
    MIN_HOVER_TIME = 0.5  # Regular waypoint hover time
    FINAL_HOVER_TIME = 2.0  # Final waypoint hover time
    
    for i in range(0, int(duration_sec * env.CTRL_FREQ)):
        obs, _, _, _, _ = env.step(action)
        
        all_drones_finished = True
        
        for j in range(num_drones):
            if wp_counters[j] < num_wps:
                all_drones_finished = False
                target_wp = waypoints[wp_counters[j]]
                action[j, :], _, _ = controllers[j].computeControlFromState(
                    control_timestep=env.CTRL_TIMESTEP,
                    state=obs[j],
                    target_pos=target_wp,
                    target_rpy=INIT_RPYS[j]
                )
                action[j, :] *= 1
                # Check if drone is hovering at waypoint
                distance_to_waypoint = np.linalg.norm(obs[j][:3] - target_wp)
                if distance_to_waypoint < HOVER_THRESHOLD:
                    waypoint_hover_time[j] += env.CTRL_TIMESTEP
                else:
                    waypoint_hover_time[j] = 0

                # Determine required hover time based on whether it's the final waypoint
                required_hover_time = FINAL_HOVER_TIME if wp_counters[j] == num_wps - 1 else MIN_HOVER_TIME

                # If hovered for required time, move to next waypoint
                if waypoint_hover_time[j] >= required_hover_time:
                    update_waypoint_color(waypoint_ids[wp_counters[j]], env)
                    wp_counters[j] += 1
                    waypoint_hover_time[j] = 0
                    
                    # Print progress
                    if wp_counters[j] < num_wps:
                        print(f"Drone {j} reached waypoint {wp_counters[j]}/{num_wps}")
                    else:
                        print(f"Drone {j} completed all waypoints!")

        env.render()
        if gui:
            sync(i, START, env.CTRL_TIMESTEP)
            
        # If all drones have completed their waypoints, terminate the simulation
        if all_drones_finished:
            print("All drones have completed their waypoints. Terminating simulation.")
            break

    env.close()
if __name__ == "__main__":
    waypoints = [(0, 0, 1), (2, 1, 2), (3, 1, 2), (2, 2, 1)]
    #waypoints = [(0, 0, 1), (1, 2, 1), (1, 0, 1), (0, 1, 1)]
    waypoint_navigation(waypoints)