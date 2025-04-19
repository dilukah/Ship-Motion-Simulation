# main.py

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from model import Ship
from controller import PIDController
from video import init_frame_folder, save_frame, export_video

# Create a folder for frames
frame_folder = init_frame_folder(folder = "frames")
frame_paths = []

simulation_done = False

# Waypoints
waypoints = [
    np.array([0, 0]),
    np.array([50, 30]),
    np.array([100, 30]),
    np.array([150, 80])
]

lookahead_distance = 10.0
arrival_threshold = 5.0
dt = 0.1
sim_time = 100

# Initialize ship model
ship = Ship(dt=dt)

# Controller
controller = PIDController()

trajectory = []
waypoint_index = 1

# Line-of-sight guidance
def compute_los_heading(pos, wp_prev, wp_next):
    path_vec = wp_next - wp_prev
    path_unit = path_vec / np.linalg.norm(path_vec)
    rel_pos = pos - wp_prev
    proj_len = np.dot(rel_pos, path_unit)
    proj_point = wp_prev + proj_len * path_unit
    lookahead = proj_point + lookahead_distance * path_unit
    return np.arctan2(lookahead[1] - pos[1], lookahead[0] - pos[0])

# Wind disturbance
def wind_force(t):
    sway_force = 30 * np.sin(0.05 * t)
    yaw_moment = 20 * np.cos(0.03 * t)
    return np.array([0.0, sway_force, yaw_moment])

# Visualization setup
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim(-20, 200)
ax.set_ylim(-20, 120)

ship_body, = ax.plot([], [], 'b-', lw=3)
trajectory_line, = ax.plot([], [], 'r--', lw=1)
waypoint_dots, = ax.plot(*zip(*waypoints), 'go', markersize=6, label="Waypoints")
current_wp_marker, = ax.plot([], [], 'ro', markersize=10, label="Target WP")
ax.legend()

def get_ship_shape():
    return np.array([[2, 0], [-2, -1], [-2, 1], [2, 0]])

def transform_shape(shape, x, y, psi):
    rot = np.array([
        [np.cos(psi), -np.sin(psi)],
        [np.sin(psi),  np.cos(psi)]
    ])
    return (rot @ shape.T).T + np.array([x, y])

def animate(frame):
    global waypoint_index, simulation_done
    
    if simulation_done:
        return []

    t = frame * dt
    pos = ship.get_position()
    psi = ship.get_heading()

    if waypoint_index < len(waypoints):
        wp_prev = waypoints[waypoint_index - 1]
        wp_next = waypoints[waypoint_index]
        dist_to_wp = np.linalg.norm(pos - wp_next)

        if dist_to_wp < arrival_threshold and waypoint_index < len(waypoints) - 1:
            waypoint_index += 1
            wp_prev = waypoints[waypoint_index - 1]
            wp_next = waypoints[waypoint_index]

        desired_heading = compute_los_heading(pos, wp_prev, wp_next)
    else:
        desired_heading = psi  # hold current heading

    wind = wind_force(t)
    yaw_control = controller.compute(psi, desired_heading, dt)
    ship_state = ship.step(yaw_control, wind)
    trajectory.append(ship_state[:2])

    shape = transform_shape(get_ship_shape(), ship_state[0], ship_state[1], ship_state[2])
    ship_body.set_data(shape[:, 0], shape[:, 1])
    traj_np = np.array(trajectory)
    trajectory_line.set_data(traj_np[:, 0], traj_np[:, 1])
    #print("Frame no: ", frame, "\n")
    if waypoint_index < len(waypoints):
        wp = waypoints[waypoint_index]
        current_wp_marker.set_data(wp[0], wp[1])
    else:
        current_wp_marker.set_data([], [])
        simulation_done = True
        print("Simulation complete")
        ani.event_source.stop()         

    # Save frame
    frame_path = save_frame(fig, frame, folder=frame_folder) 
    frame_paths.append(frame_path)

    return ship_body, trajectory_line, current_wp_marker

frames = int(sim_time / dt)
ani = FuncAnimation(fig, animate, frames=frames, interval=dt*1000, repeat=False)
plt.title("Ship Motion Simulation")
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.grid(True)
plt.show()


# Save frames as video
video_path = "Ship_simulation.mp4"
export_video(frame_paths, output_path=video_path, fps=20, cleanup=True)
