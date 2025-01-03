import csv
import numpy as np
import modern_robotics as mr
import milestone1
import milestone2
import milestone_3

# Function to save a trajectory to a CSV file
def save_trajectory(filename, data):
    """
    Save trajectory data to a CSV file.

    Parameters:
        filename: Name of the output file.
        data: List of data rows to be written to the file.
    """
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(data)


if __name__ == "__main__":
    """
    Example of how to run the code:
    Save this script as `main_script.py` and ensure all required modules 
    (`milestone1.py`, `milestone2.py`, `milestone_3.py`) 
    are in the same directory. Adjust the below parameters to your liking, then run the following command in the terminal:

        python main_script.py

    This script generates two outputs:
    - `final_trajectory.csv`: Configuration trajectory for loading into CoppeliaSim.
    - `Xerr_log.csv`: A log of the error vector over time for analyzing convergence.
    """

    # -----------------------------
    # USER-SPECIFIED INPUTS
    # -----------------------------

    # Initial cube configuration in the space frame
    T_sc_initial = np.array([
        [1, 0, 0, 1],  # Cube at (1, 0, 0.025)
        [0, 1, 0, 0],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])

    # Final cube configuration in the space frame
    T_sc_final = np.array([
        [0, 1, 0, 0],  # Cube at (0, -1, 0.025)
        [-1, 0, 0, -1],
        [0, 0, 1, 0.025],
        [0, 0, 0, 1]
    ])

    # Initial robot configuration: [phi, x, y, J1, J2, J3, J4, J5, W1, W2, W3, W4, gripper]
    initial_config = [
        0.6, -1, -0.5,   
        -0.5, -0.6, -0.755, -0.2, -0.75,  
        0, 0, 0, 0,       
        0                 
    ]

    # Initial end-effector configuration for the reference trajectory
    T_se_initial = np.array([
        [0, 0, 1, 0],  # End-effector starts above the cube
        [0, 1, 0, 0],
        [-1, 0, 0, 0.5],
        [0, 0, 0, 1]
    ])

    # Feedback control gains for proportional (Kp) and integral (Ki) control
    Kp = 1 * np.eye(6)  
    Ki = 3 * np.eye(6)  
    dt = 0.01           # Time step for simulation
    max_speed = 50      # Maximum allowable speed for wheels and joints (rad/s)

    # Define transformation matrices for the end-effector's grasp and standoff poses
    theta = np.deg2rad(135)  # Rotate 135 degrees around the y-axis
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    R_y = np.array([  # Rotation matrix around the y-axis
        [cos_theta, 0, sin_theta],
        [0, 1, 0],
        [-sin_theta, 0, cos_theta]
    ])

    T_ce_standoff = np.array([
        [R_y[0, 0], R_y[0, 1], R_y[0, 2], 0],
        [R_y[1, 0], R_y[1, 1], R_y[1, 2], 0],
        [R_y[2, 0], R_y[2, 1], R_y[2, 2], 0.03],  # 3 cm above the block
        [0, 0, 0, 1]
    ])

    T_ce_grasp = np.array([
        [R_y[0, 0], R_y[0, 1], R_y[0, 2], 0],
        [R_y[1, 0], R_y[1, 1], R_y[1, 2], 0],
        [R_y[2, 0], R_y[2, 1], R_y[2, 2], 0],  
        [0, 0, 0, 1]
    ])

    # Generate the reference trajectory for the end-effector
    k = 1  # Configurations per 0.01 seconds
    ref_traj = milestone2.TrajectoryGenerator(T_se_initial, T_sc_initial, T_sc_final, T_ce_grasp, T_ce_standoff, k)

    # Odometry parameters for the mobile base
    r = 0.0475  # Wheel radius (m)
    l = 0.235   # Distance from the center of the robot to the wheels along the x-axis
    w = 0.15    # Distance from the center of the robot to the wheels along the y-axis

    # Compute F matrix 
    F = (r / 4) * np.array([
        [-1 / (l + w), 1 / (l + w), 1 / (l + w), -1 / (l + w)],
        [1, 1, 1, 1],
        [-1, 1, -1, 1]
    ])
    F6 = np.vstack((np.zeros((2, 4)), F, np.zeros((1, 4))))  # Extend F to a 6x4 matrix

    # Transformation matrices for the arm's kinematics
    T_b0 = np.array([  # From the base frame to the arm base frame
        [1, 0, 0, 0.1662],
        [0, 1, 0, 0],
        [0, 0, 1, 0.0026],
        [0, 0, 0, 1]
    ])

    M0e = np.array([  # Home configuration of the end-effector relative to the arm base
        [1, 0, 0, 0.033],
        [0, 1, 0, 0],
        [0, 0, 1, 0.6546],
        [0, 0, 0, 1]
    ])

    Blist = np.array([  # Screw axes in the end-effector frame at home
        [0, 0, 1, 0, 0.033, 0],
        [0, -1, 0, -0.5076, 0, 0],
        [0, -1, 0, -0.3526, 0, 0],
        [0, -1, 0, -0.2176, 0, 0],
        [0, 0, 1, 0, 0, 0]
    ]).T

    # Simulation parameters
    N = len(ref_traj)  # Total number of trajectory configurations
    steps = N - 1      # Number of control steps
    current_config = np.array(initial_config)  # Initialize robot configuration
    Xerr_integral = np.zeros(6)  # Initialize integral error

    config_log = []  # Log of all configurations
    Xerr_log = []    # Log of all error vectors

    # Log initial configuration
    current_gripper = ref_traj[0][-1]
    config_log.append(current_config.tolist())

    for i in range(steps):
        # Extract current and next desired configurations
        Xd_vec = ref_traj[i]
        Xd_next_vec = ref_traj[i + 1]

        R_d = np.array(Xd_vec[0:9]).reshape(3, 3)  # Rotation matrix of Xd
        p_d = np.array(Xd_vec[9:12])              # Position vector of Xd
        g_d = Xd_vec[12]                          # Gripper state
        Xd = np.eye(4)
        Xd[0:3, 0:3] = R_d
        Xd[0:3, 3] = p_d

        R_d_next = np.array(Xd_next_vec[0:9]).reshape(3, 3)
        p_d_next = np.array(Xd_next_vec[9:12])
        Xd_next = np.eye(4)
        Xd_next[0:3, 0:3] = R_d_next
        Xd_next[0:3, 3] = p_d_next

        # Compute current end-effector pose
        X = milestone_3.compute_end_effector_pose(current_config, Blist, M0e, T_b0)

        # Feedback control to compute twist V
        V, Xerr_integral_new = milestone_3.FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_integral)
        X_err_mat = mr.MatrixLog6(mr.TransInv(X) @ Xd)  # Error twist matrix
        X_err = mr.se3ToVec(X_err_mat)                  # Error vector
        Xerr_integral = Xerr_integral_new               # Update error integral

        # Compute wheel and joint speeds with joint limits
        u, joint_speeds = milestone_3.compute_controls_with_limits(V, current_config, F6, T_b0, M0e, Blist, dt, 1e-3)
        controls = np.concatenate([u, joint_speeds])  # Combine wheel and joint speeds

        # Update robot configuration
        current_config = milestone1.NextState(current_config, controls, dt, max_speed)

        # Log the current configuration and error
        config_log.append(current_config.tolist() + [g_d])
        Xerr_log.append(X_err.tolist())

    # Save configuration trajectory and error logs
    save_trajectory("final_trajectory.csv", config_log)
    save_trajectory("Xerr_log.csv", Xerr_log)

    print("Simulation complete. Files saved:")
    print("- final_trajectory.csv (Load into CoppeliaSim)")
    print("- Xerr_log.csv (Plot for error analysis)")
