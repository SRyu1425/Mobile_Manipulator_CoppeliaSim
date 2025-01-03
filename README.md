# Mobile_Manipulator_CoppeliaSim

This project implements a software solution to control the motion of a mobile manipulator (youBot) to perform a block pick-and-place task. The software generates a reference trajectory for the end-effector, computes feedback controls to track the trajectory, and enforces joint limits to avoid self-collisions and singularities.
The robot's motion is simulated, and the results are saved in CSV files for visualization in CoppeliaSim. The project integrates concepts like trajectory generation, feedback control, and kinematic modeling.

## Python Program

1. Main Features
1. Trajectory Generation:
○ The TrajectoryGenerator function computes a smooth reference trajectory using quintic time scaling for the youBot's end-effector.
○ The trajectory includes intermediate poses for standoff, grasp, and release configurations.
2. Feedback Control:
○ A feedback controller calculates the commanded end-effector twist (V) using proportional-integral (PI) control to track the desired trajectory.
○ Gains (Kp and Ki) can be customized to optimize performance.
3. Enforcing Joint Limits:
○ Joint limits are defined to avoid self-collisions and singularities.
○ The compute_controls_with_limits function modifies the Jacobian matrix when joint limits are violated, ensuring safe operation.
○ An extra video without limits is included in the “best” directory of “results for comparison
4. Simulation:
○ The NextState function updates the robot's configuration over time using control inputs, respecting maximum speed constraints.
5. Logging:
○ The final trajectory and error data are saved as final_trajectory.csv and Xerr_log.csv for evaluation and visualization.
