# eye_to_hand_calibration_example
In-class example for eye-to-hand calibration

**NOTE**

This example uses data from [1]. However, this is intended for a hand-in-hand calibration. Results will not be physically accurate.

## Downloading Code
1. Navigate to examples workspace source folder: `cd ~/workspaces/examples_ws/src`
2. Download code: `git clone https://github.com/Vanderbilt-Applied-Robotics-Lab/eye_to_hand_calibration_example`

## Compiling Code
1. Navigate to examples workspace: `cd ~/workspaces/examples_ws`
2. Compile the code: `colcon build`

## Running Code
1. Navigate to the examples workspace: `cd ~/workspaces/examples_ws`
2. Source the examples workspace: `source install/setup.bash`
3. Run the code: `ros2 run eye_hand_calib eye_hand_calib`

## References
[1]  Tabb, A., Ahmad Yousef, K.M. Solving the robot-world hand-eye(s) calibration problem with iterative methods. Machine Vision and Applications 28, 569–590 (2017).  https://doi.org/10.1007/s00138-017-0841-7



