# Mobile_Manipulator_CoppeliaSim

This project implements a software solution to control the motion of a mobile manipulator (youBot) to perform a block pick-and-place task. The software generates a reference trajectory for the end-effector, computes feedback controls to track the trajectory, and enforces joint limits to avoid self-collisions and singularities.
The robot's motion is simulated, and the results are saved in CSV files for visualization in CoppeliaSim. The project integrates concepts like trajectory generation, feedback control, and kinematic modeling.

## Python Program

### 1. Main Features

- Trajectory Generation:
  - The TrajectoryGenerator function computes a smooth reference trajectory using quintic time scaling for the youBot's end-effector.
  - The trajectory includes intermediate poses for standoff, grasp, and release configurations.
- Feedback Control:
  - A feedback controller calculates the commanded end-effector twist (V) using proportional-integral (PI) control to track the desired trajectory.
  - Gains (Kp and Ki) can be customized to optimize performance.
- Enforcing Joint Limits:
  - Joint limits are defined to avoid self-collisions and singularities.
  - The compute_controls_with_limits function modifies the Jacobian matrix when joint limits are violated, ensuring safe operation.
  - An extra video without limits is included in the “best” directory of “results for comparison
- Simulation:
  - The NextState function updates the robot's configuration over time using control inputs, respecting maximum speed constraints.
- Logging:
  - The final trajectory and error data are saved as final_trajectory.csv and Xerr_log.csv for evaluation and visualization.

### 2. File Descriptions

- Main Script
  - The main script integrates all components (trajectory generation, feedback control, and simulation) to perform the task.
- Supporting Modules:
  - milestone1.py: Implements the NextState function to update the robot's configuration.
  - milestone2.py: Contains the TrajectoryGenerator function.
  - milestone_3.py: Implements joint limit handling and feedback control.
- Generated Files:
  - final_trajectory.csv: Contains the robot's trajectory, which can be loaded into CoppeliaSim for visualization.
  - Xerr_log.csv: Logs the end-effector error over time.
 
## Results

- Trajectory Execution:
  - The robot successfully executes the pick-and-place task in CoppeliaSim using the generated trajectory.
  - The block is picked from its initial position and placed at the target position without collision or singularity issues.
- Error Convergence:
  - The error (6-vector) converges over time, ensuring accurate tracking of the trajectory.
- Joint Limits:
  - The implementation of joint limits prevents self-collisions and singularities, allowing the robot to operate realistically.
 
## Challenges and Insights

- Challenges:
  - At first, I did not realize that there needs to be a sufficient Ki term to induce overshoot and oscillations. I mistakenly thought that simply having a Kp term high enough would create overshoot.
  - Finding the right combination of joint limits and a grasping configuration for the block required many trial and error attempts
- Insights:
  - In the trajectories that I tested, incorporating a singularity tolerance on the Jacobian pseudoinverse did not seem to have any noticeable effect, so I kept it at a reasonable 1e-3.
 
## Joint Limits

- Method:
  - Joint angles were checked against predefined limits before updating the configuration.
  - If any limits were violated, the corresponding columns in the Jacobian matrix were zeroed out, preventing motion in the violating joints.
  - The pseudoinverse of the modified Jacobian was recalculated to compute safe control commands.
- Choosing Limits:
  - Limits were found by testing different arm joint angle slider combinations in Scene 3 and ensuring the arm could reach the necessary poses by not overly constraining its workspace
  - The robot was tested to avoid any self collisions or singularities within the joint limits chosen
