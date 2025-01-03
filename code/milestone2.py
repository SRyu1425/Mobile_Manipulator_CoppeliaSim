import numpy as np    
import csv            
import modern_robotics as mr  

def TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k):
    """
    Generate a reference trajectory for the youBot end-effector

    Parameters:
        T_se_initial : initial end-effector configuration in the space frame
        T_sc_initial : initial cube configuration in the space frame
        T_sc_final : desired final cube configuration in the space frame
        T_ce_grasp : end-effector's grasp configuration relative to the cube frame
        T_ce_standoff : standoff configuration relative to the cube frame
        k : Number of trajectory reference configurations per 0.01 seconds

    Returns:
        trajectory (list): List of 13-element lists representing the trajectory configurations
                           Each element consists of the flattened rotation matrix (9 elements),
                           the position vector (3 elements), and the gripper state (1 element)
    """
    # Calculate key frames in the space frame 
    T_se_standoff_initial = np.dot(T_sc_initial, T_ce_standoff)  # End-effector standoff position above the cube before grasping
    T_se_grasp = np.dot(T_sc_initial, T_ce_grasp)                # End-effector position when grasping the cube
    T_se_standoff_final = np.dot(T_sc_final, T_ce_standoff)      # End-effector standoff position above the goal location
    T_se_release = np.dot(T_sc_final, T_ce_grasp)                # End-effector position when releasing the cube

    # Trajectory segment durations (in seconds)
    durations = {
        "move_to_initial_standoff": 7.0,  #  move from initial position to standoff above the cube
        "lower_to_grasp": 1.0,            #  lower from standoff to grasp position
        "grasp_block": 0.625,             #  close the gripper
        "lift_to_standoff": 1.0,          #  lift from grasp position back to standoff
        "move_to_final_standoff": 7.0,    #  move from initial standoff to standoff above the goal
        "lower_to_release": 1.0,          #  lower from standoff to release position
        "release_block": 0.625,           #  open the gripper
        "lift_back_to_standoff": 1.0      #  lift from release position back to standoff
    }

    # Calculate total points per segment based on durations and k
    num_points = {key: int(value * k * 100) for key, value in durations.items()}
    # Each segment duration is multiplied by k (configurations per 0.01s) and 100 (to convert seconds to 0.01s intervals)

    method = 5  # Quintic time scaling

    # -------------------- Trajectory Segments --------------------

    # Segment 1: Move from initial configuration to standoff position above the cube
    traj1 = mr.CartesianTrajectory(
        T_se_initial, T_se_standoff_initial,
        durations["move_to_initial_standoff"], num_points["move_to_initial_standoff"], method
    )
    gripper_state_1 = [0] * num_points["move_to_initial_standoff"]  # Gripper is open (0) during this segment

    # Segment 2: Lower end-effector from standoff position to grasp position
    traj2 = mr.CartesianTrajectory(
        T_se_standoff_initial, T_se_grasp,
        durations["lower_to_grasp"], num_points["lower_to_grasp"], method
    )
    gripper_state_2 = [0] * num_points["lower_to_grasp"]  # Gripper remains open

    # Segment 3: Grasp the block (end-effector remains stationary while gripper closes)
    grasp = [T_se_grasp] * num_points["grasp_block"]  # Repeat the grasp pose
    gripper_state_3 = [1] * num_points["grasp_block"]  # Gripper closes (1)

    # Segment 4: Lift the block back to standoff position
    traj4 = mr.CartesianTrajectory(
        T_se_grasp, T_se_standoff_initial,
        durations["lift_to_standoff"], num_points["lift_to_standoff"], method
    )
    gripper_state_4 = [1] * num_points["lift_to_standoff"]  # Gripper remains closed

    # Segment 5: Move from initial standoff to final standoff position above the goal
    traj5 = mr.CartesianTrajectory(
        T_se_standoff_initial, T_se_standoff_final,
        durations["move_to_final_standoff"], num_points["move_to_final_standoff"], method
    )
    gripper_state_5 = [1] * num_points["move_to_final_standoff"]  # Gripper remains closed

    # Segment 6: Lower end-effector from standoff to release position
    traj6 = mr.CartesianTrajectory(
        T_se_standoff_final, T_se_release,
        durations["lower_to_release"], num_points["lower_to_release"], method
    )
    gripper_state_6 = [1] * num_points["lower_to_release"]  # Gripper remains closed

    # Segment 7: Release the block (end-effector remains stationary while gripper opens)
    release = [T_se_release] * num_points["release_block"]  # Repeat the release pose
    gripper_state_7 = [0] * num_points["release_block"]  # Gripper opens (0)

    # Segment 8: Lift end-effector back to standoff position after releasing the block
    traj8 = mr.CartesianTrajectory(
        T_se_release, T_se_standoff_final,
        durations["lift_back_to_standoff"], num_points["lift_back_to_standoff"], method
    )
    gripper_state_8 = [0] * num_points["lift_back_to_standoff"]  # Gripper remains open

    # -------------------- Combine Trajectories and Gripper States --------------------

    # Concatenate all trajectory segments to form the full trajectory
    full_trajectory = traj1 + traj2 + grasp + traj4 + traj5 + traj6 + release + traj8

    # Concatenate all gripper state lists to form the full gripper state sequence
    full_gripper_states = (
        gripper_state_1 + gripper_state_2 + gripper_state_3 +
        gripper_state_4 + gripper_state_5 + gripper_state_6 +
        gripper_state_7 + gripper_state_8
    )

    # -------------------- Format Trajectory for CSV Output --------------------

    trajectory = [] 
    for T, gripper_state in zip(full_trajectory, full_gripper_states):
        # Extract rotation matrix and flatten it into a list
        R = T[:3, :3].flatten().tolist()  # Rotation matrix elements (9 elements)
        # Extract position vector
        p = T[:3, 3].tolist()             # Position vector elements (3 elements)
        # Append the flattened rotation matrix, position vector, and gripper state to the trajectory list
        trajectory.append(R + p + [gripper_state])  # Total of 13 elements per configuration

    return trajectory

def save_trajectory_to_csv(filename, trajectory):
    """
    Save the trajectory to a CSV file

    Parameters:
        filename (str): Name of the output CSV file
        trajectory (list): Trajectory data with 13 elements per row
    """
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)  
        writer.writerows(trajectory) 

if __name__ == "__main__":
    # -------------------- Inputs --------------------

    # Compute the rotation matrix for gripper position
    theta = np.deg2rad(135)  
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    R_y = np.array([
        [cos_theta, 0, sin_theta],
        [0,         1, 0        ],
        [-sin_theta, 0, cos_theta]
    ])

    # Define the standoff configuration relative to the cube frame
    T_ce_standoff = np.array([
        [R_y[0, 0], R_y[0, 1], R_y[0, 2], 0],    
        [R_y[1, 0], R_y[1, 1], R_y[1, 2], 0],    
        [R_y[2, 0], R_y[2, 1], R_y[2, 2], 0.03],  # z-offset (0.03 m standoff height)
        [0,         0,         0,         1   ]   
    ])

    # Define the initial end-effector configuration in the space frame
    T_se_initial = np.array([
        [0, 0, 1, 0.48],       
        [0, 1, 0, 0],      
        [-1, 0, 0, 0.528],    
        [0, 0, 0, 1]
    ])

    # Define the initial cube configuration in the space frame
    T_sc_initial = np.array([
        [1, 0, 0, 1],      # (x,y,z) = (1,0,0.025) 
        [0, 1, 0, 0],       
        [0, 0, 1, 0.025],    
        [0, 0, 0, 1]
    ])

    # Define the desired final cube configuration in the space frame
    T_sc_final = np.array([
        [0, 1, 0, 0],       # (x,y,z) = (0, -1, 0.025)  
        [-1, 0, 0, -1],  
        [0, 0, 1, 0.025],    
        [0, 0, 0, 1]
    ])

    # Define the end-effector's grasp configuration relative to the cube frame
    T_ce_grasp = np.array([
        [R_y[0, 0], R_y[0, 1], R_y[0, 2], 0],
        [R_y[1, 0], R_y[1, 1], R_y[1, 2], 0], 
        [R_y[2, 0], R_y[2, 1], R_y[2, 2], 0],  
        [0,         0,         0,         1 ]  
    ])

    k = 1  # Number of trajectory reference configurations per 0.01 seconds

    # -------------------- Generate and Save Trajectory --------------------

    # Generate the trajectory using the defined configurations and parameters
    trajectory = TrajectoryGenerator(
        T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k
    )

    # Save the generated trajectory to a CSV file
    save_trajectory_to_csv("trajectory.csv", trajectory)

    #print("Trajectory generation complete. Saved to 'trajectory.csv'.")
