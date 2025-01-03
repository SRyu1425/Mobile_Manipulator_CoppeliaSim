import numpy as np
import modern_robotics as mr

def compute_end_effector_pose(current_config, Blist, M0e, Tb0):
    """
    Compute the current end-effector pose X from the robot configuration.
    current_config: [phi, x, y, theta1...theta5, w1...w4]
    Blist: 6x5 matrix of arm screw axes in end-effector frame at home
    M0e: Home configuration of end-effector in arm base frame
    Tb0: Transformation from chassis base to arm base
    """
    phi, x, y = current_config[0], current_config[1], current_config[2]

    # Chassis configuration
    Tsb = np.array([
        [np.cos(phi), -np.sin(phi), 0, x],
        [np.sin(phi),  np.cos(phi), 0, y],
        [0,            0,           1, 0.0963],
        [0,            0,           0, 1]
    ])

    # Arm configuration
    thetalist = current_config[3:8]

    # End-effector pose using forward kinematics
    T0e = mr.FKinBody(M0e, Blist, thetalist)
    X = Tsb @ Tb0 @ T0e
    return X

def FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, Xerr_integral):
    """
    Computes the commanded end-effector twist V using the feedforward-plus-feedback control law.
    
    Parameters:
        X (4x4): Current end-effector configuration
        Xd (4x4): Current reference end-effector configuration
        Xd_next (4x4): Next reference end-effector configuration (after dt time)
        Kp (6x6): Proportional gain matrix
        Ki (6x6): Integral gain matrix
        dt: Timestep
        Xerr_integral (6-vec): Integral of the error so far

    Returns:
        V (6-vec): The commanded end-effector twist in the end-effector frame
        Xerr_integral_new (6-vec): Updated integral of the error
    """
    
    # Compute error twist that takes X to Xd
    X_err_mat = mr.MatrixLog6(mr.TransInv(X) @ Xd)
    X_err = mr.se3ToVec(X_err_mat)

    # Update integral error
    Xerr_integral_new = Xerr_integral + X_err * dt

    # Compute feedforward reference twist Vd that takes Xd to Xd_next in dt
    Vd_mat = mr.MatrixLog6(mr.TransInv(Xd) @ Xd_next)
    Vd = mr.se3ToVec(Vd_mat) / dt


    # Compute the adjoint matrix Ad_{X^{-1}Xd}
    Ad_term = mr.Adjoint(mr.TransInv(X) @ Xd)

    #print("Adjoint @ Vd term!")
    #print(np.round(Ad_term @ Vd, decimals=3))

    # Control law: V = Ad_{X^{-1}X_d} Vd + Kp * X_err + Ki * Xerr_integral
    V = Ad_term @ Vd + Kp @ X_err + Ki @ Xerr_integral_new
    #non feedforward
    #V = Kp @ X_err + Ki @ Xerr_integral_new

    return V, Xerr_integral_new

def testJointLimits(joint_angles, joint_speeds, dt):
    """
    Check if the next configuration of the arm joints will violate joint limits.

    Parameters:
        joint_angles (list or np.array): Current joint angles (theta1, ..., theta5).
        joint_speeds (list or np.array): Proposed joint angular velocities (theta1_dot, ..., theta5_dot).
        dt (float): Time step for simulation.

    Returns:
        violating_indices (list): List of indices of joints that violate their limits.
    
    """
    # Define the joint angle limits for the robot arm.
    # These limits are chosen to avoid singularities and self-collisions.
    joint_limits = [
        (-np.pi / 2, np.pi / 2),  # Joint 1: [-90, 90] degrees
        (-1.5, -0.05),            # Joint 2: [-1.5, -0.05] radians
        (-1.6, -0.05),            # Joint 3: [-1.6, -0.05] radians
        (-0.35, -0.05),           # Joint 4: [-0.35, -0.05] radians
        (-np.pi / 2, np.pi / 2)   # Joint 5: [-90, 90] degrees
    ]

    # Predict the next joint angles using the current angles, speeds, and time step.
    next_angles = joint_angles + joint_speeds * dt

    # Identify any joints that would violate their limits.
    violating_indices = []
    for i, (lower, upper) in enumerate(joint_limits):
        if not (lower <= next_angles[i] <= upper):
            violating_indices.append(i)  # Add index if the limit is violated.

    return violating_indices


def compute_controls_with_limits(V, current_config, F6, T_b0, M0e, Blist, dt, tolerance=1e-3):
    """
    Compute wheel and arm joint speeds to achieve the desired end-effector twist (V),
    while enforcing joint limits to avoid singularities and self-collisions.

    Parameters:
        V (np.array): Desired end-effector twist (6D vector).
        current_config (np.array): Current robot configuration (12-vector: [phi, x, y, theta1-5, wheel1-4]).
        F6 (np.array): Coupling matrix for chassis movement (6x4 matrix).
        T_b0 (np.array): Transformation from the base frame to the arm base frame (4x4 matrix).
        M0e (np.array): Home configuration of the end-effector in the arm base frame (4x4 matrix).
        Blist (np.array): Screw axes for the arm in the end-effector frame at home (6x5 matrix).
        dt (float): Time step for simulation.
        tolerance (float): Numerical tolerance for pseudoinverse computation.

    Returns:
        u (np.array): Wheel speeds (4D vector).
        joint_speeds (np.array): Joint speeds (5D vector).
    """
    # Extract the current joint angles from the robot configuration.
    joint_angles = current_config[3:8]

    # Compute the full Jacobian (Je), which combines the chassis and arm contributions.
    # The Jacobian helps map joint and wheel speeds to end-effector motion.
    T_0e = mr.FKinBody(M0e, Blist, joint_angles)  # Compute current end-effector pose in the arm frame.
    T_eb = mr.TransInv(T_0e) @ mr.TransInv(T_b0)  # Transform from base frame to end-effector frame.
    Ad_Teb = mr.Adjoint(T_eb)                    # Adjoint transformation matrix.
    J_base = Ad_Teb @ F6                         # Base (chassis) Jacobian (6x4 matrix).
    J_arm = mr.JacobianBody(Blist, joint_angles) # Arm Jacobian (6x5 matrix).
    Je = np.hstack((J_base, J_arm))              # Combine base and arm Jacobians (6x9 matrix).

    # Compute wheel and joint speeds using the pseudoinverse of the Jacobian.
    speeds = np.linalg.pinv(Je, rcond=tolerance) @ V  # Solve for speeds that achieve the desired twist.
    u = speeds[:4]                                  # Extract wheel speeds (first 4 elements).
    joint_speeds = speeds[4:]                       # Extract joint speeds (remaining 5 elements).

    # Check if the proposed joint speeds violate any joint limits.
    violating_indices = testJointLimits(joint_angles, joint_speeds, dt)

    if len(violating_indices) > 0:  # If any limits are violated:
        # Create a modified Jacobian by zeroing out the columns corresponding to violating joints.
        J_modified = Je.copy()
        for idx in violating_indices:
            arm_col = 4 + idx  # Map joint index to the corresponding column in Je.
            J_modified[:, arm_col] = 0.0  # Zero out the column for this joint.

        # Recompute speeds using the modified Jacobian.
        speeds = np.linalg.pinv(J_modified, rcond=tolerance) @ V
        u = speeds[:4]           # Extract updated wheel speeds.
        joint_speeds = speeds[4:]  # Extract updated joint speeds.

    return u, joint_speeds  # Return the computed wheel and joint speeds.