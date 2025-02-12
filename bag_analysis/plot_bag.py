import sys
import rosbag
import numpy as np
import matplotlib.pyplot as plt

def plot(filename, robot_name):
    bag = rosbag.Bag(filename)
    messages = list(bag.read_messages(topics=[f"/{robot_name}/wheels_driver_node/wheels_cmd"]))
    radius = 0.0318
    baseline = 0.1
    omega_max = 8.0

    xi_world_frame = np.array([[0], [0], [np.pi/2]])
    theta = np.pi/2

    x_world_values = []
    y_world_values = []
    theta_world_values = []

    for i, bag_message in enumerate(messages):
        if i == len(messages)-1:
            break
        msg = bag_message.message
        next_msg = messages[i+1].message

        current_stamp = msg.header.stamp
        next_stamp = next_msg.header.stamp
        dt = (next_stamp - current_stamp).to_sec()

        vel_left = msg.vel_left * omega_max
        vel_right = msg.vel_right * omega_max

        
        change_robot_frame = np.array([[(radius * vel_right)/2 + (radius * vel_left)/2], 
                                        [0], 
                                        [(radius * (vel_right - vel_left)) / baseline]])
        
        theta += (change_robot_frame[2, 0]*dt)
        
        change_in_xi_world_frame = np.array([[np.cos(theta), -np.sin(theta), 0], 
                                       [np.sin(theta), np.cos(theta), 0], 
                                       [0, 0, 1]]) @ change_robot_frame
        
        xi_world_frame += change_in_xi_world_frame*dt
        
        x_world_values.append(xi_world_frame[0, 0])
        y_world_values.append(xi_world_frame[1, 0])
        theta_world_values.append(xi_world_frame[2, 0])


    bag.close()
    plt.figure(figsize=(8, 6))
    plt.plot(x_world_values, y_world_values, label="Robot Path", marker="o", markersize=1, linestyle="-")
    print(min(x_world_values + y_world_values), max(x_world_values + y_world_values))
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Robot Trajectory in World Frame")
    plt.legend()
    plt.grid()
    plt.axis("equal")  # Ensure equal scaling for x and y
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) >= 4 or len(sys.argv) < 3:
        raise ValueError("Please the rosbag file and the ROBOT_NAME")
    plot(sys.argv[1], sys.argv[2])