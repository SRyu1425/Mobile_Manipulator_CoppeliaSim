import csv
import numpy as np
import modern_robotics as mr  

def NextState(current_config, controls, dt, max_speed=999):
    """
    Compute the next state of the youBot after a time dt, given the current configuration and controls.
    
    Parameters:
        current_config: 12 vector- [phi, x, y, theta1, theta2, theta3, theta4, theta5, w1, w2, w3, w4]
        controls:  9 vector - [u1, u2, u3, u4, joint1_dot, joint2_dot, joint3_dot, joint4_dot, joint5_dot]
        dt: Timestep
        max_speed: Maximum allowed angular speed for wheels and joints (rad/s)
        
    Returns:
        next_config: The new 12 vector configuration after dt.
    """
    # Extract current state
    phi, x, y = current_config[0], current_config[1], current_config[2]
    arm_angles = np.array(current_config[3:8])   # theta1,...,theta5
    wheel_angles = np.array(current_config[8:12]) # w1,...,w4

    # Extract controls
    wheel_speeds = np.array(controls[0:4])   # u1,...,u4
    joint_speeds = np.array(controls[4:9])   # joint1_dot,...,joint5_dot

    # Cap off speeds to max_speed
    wheel_speeds = np.clip(wheel_speeds, -max_speed, max_speed)
    joint_speeds = np.clip(joint_speeds, -max_speed, max_speed)

    # Update arm joints and wheels by simple Euler
    new_arm_angles = arm_angles + joint_speeds * dt
    new_wheel_angles = wheel_angles + wheel_speeds * dt

    # Odometry parameters
    r = 0.0475
    l = 0.235
    w = 0.15

    # Compute chassis twist from wheel speeds
    #first calculate F matrix (pseudoinverse H)
    F = (r/4)*np.array([[-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                        [1,         1,       1,        1      ],
                        [-1,        1,      -1,        1      ]])
    Vb = F @ wheel_speeds

    #Vb6 = [0,0,Vb[0], Vb[1], Vb[2],0] # 6 dim version of planar twist Vb
    #T_b_bprime = mr.MatrixExp6(mr.VecTose3(Vb6)) #matrix exp =transformation matrix which expresses new chassis frame rel to initial frame

    if Vb[0] == 0:
        delta_qb = np.array([0, Vb[1] * dt, Vb[2] * dt]) #no rotation case
    else: #rotation case
        delta_qb = np.array([
        Vb[0] * dt,
        (Vb[1] * np.sin(Vb[0] * dt) + Vb[2] * (np.cos(Vb[0] * dt) - 1)) / Vb[0],
        (Vb[2] * np.sin(Vb[0] * dt) + Vb[1] * (1 - np.cos(Vb[0] * dt))) / Vb[0]
    ])

    # Rotate delta_qb from body frame to space frame using the current phi
    R = np.array([
        [1,          0,          0        ],
        [0,  np.cos(phi), -np.sin(phi)],
        [0,  np.sin(phi),  np.cos(phi)]
    ])

    delta_q = R @ delta_qb

    # delta_q = [delta_phi, delta_x, delta_y]
    new_phi = phi + delta_q[0]
    new_x   = x   + delta_q[1]
    new_y   = y   + delta_q[2]

    # Construct next configuration
    next_config = np.array([new_phi, new_x, new_y] + new_arm_angles.tolist() + new_wheel_angles.tolist())

    return next_config

def simulate_and_save(initial_config, controls, dt=0.01, total_time=1.0, max_speed=999, csv_filename="nextstate_test.csv"):
    """
    Simulate the youBot motion for total_time with given controls and save the trajectory to a CSV file.
    Each line of CSV: phi, x, y, theta_1,...,theta_5, w_1,...,w_4, gripper_state
    """
    steps = int(total_time/dt)
    config = initial_config.copy()

    # Open CSV file
    with open(csv_filename, 'w', newline='') as file:
        writer = csv.writer(file)
        
        # Write initial configuration line (with gripper=0)
        row = config.tolist() + [0]
        writer.writerow(row)

        # Simulate
        for _ in range(steps):
            config = NextState(config, controls, dt, max_speed)
            row = config.tolist() + [0]  # gripper state = 0
            writer.writerow(row)

    #print(f"Simulation complete. Results saved to {csv_filename}")

# ---------------------- TESTING ----------------------
if __name__ == "__main__":
    # Initial configuration: phi=0, x=0, y=0, all arm and wheel angles = 0
    initial_config = np.zeros(12)

    # Test 1: u=(10,10,10,10), joint speeds=0
    controls_forward = [10,10,10,10, 0,0,0,0,0]
    simulate_and_save(initial_config, controls_forward, csv_filename="forward_test.csv")

    # Test 2: u=(-10,10,-10,10), joint speeds=0
    controls_side = [-10,10,-10,10, 1,4,1,4,3]
    simulate_and_save(initial_config, controls_side, csv_filename="sideways_test.csv")

    # Test 3: u=(-10,10,10,-10), joint speeds=0
    controls_spin = [-10,10,10,-10, 0,0,0,0,0]
    simulate_and_save(initial_config, controls_spin, csv_filename="spin_test.csv")

    # Optionally, test with speed limits
    # For example, max_speed=5 should halve the distance traveled in these tests.
    simulate_and_save(initial_config, controls_forward, max_speed=5, csv_filename="forward_test_limited.csv")
