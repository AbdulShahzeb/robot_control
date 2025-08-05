# Robot Control Package

This package provides ROS 2 nodes for controlling the UR10e, acting as an interface between high-level commands and the robot's hardware drivers.

## Getting Started

### Prerequisites
* A system with ROS 2 Humble installed and configured.
* Python 3.10 environment.

### Installation

1.  **Install Python Development Headers:**
    This is required for compiling some Python packages that include C extensions.
    ```shell
    sudo apt install python3-dev python3-pip
    ```
2.  **Upgrade Pip and Build Tools:**
    Ensure you have the latest versions of pip, setuptools, and wheel to avoid common build issues.
    ```shell
    pip install --upgrade pip setuptools wheel
    ```
3.  Create or navigate to your ROS 2 workspace.
4.  Clone this repository into the `src` directory.
5.  Install required pip modules.
    ```shell
    pip install -r requirements.txt
    pip install --force numpy==1.26.4 scipy==1.11.4
    ```

6.  Navigate to the root of the repository and execute the install the URDFs/Xacros for RoboticsToolbox:
    ```shell
    ./install_ur_models.sh
    ```
7.  Build the package from the root of your workspace:
    ```shell
    colcon build --packages-select move_robot
    ```
8.  Source the environment to make the nodes available:
    ```shell
    source install/setup.bash
    ```

### ⚠️ **Critical Safety Checks**

**IMPORTANT:** Before proceeding to launch any nodes, you must read and perform the mandatory safety checks:

* **Joint Order Verification:** You must ensure the joint order defined in the `move_ur` control node correctly matches the order published on the `/joint_states` topic by the robot's driver. An incorrect order will cause erratic movements.
* **Custom Model Installation:** Confirm that the custom UR10e model files have been correctly copied into the `roboticstoolbox` Python library. The `install_ur_models.sh` script is designed to handle this, but its success should be verified. These models are crucial for accurate inverse kinematics.

### Launch

```shell
ros2 launch move_robot move_ur.launch.py
```

# Core Functionality

The node operates in a feedback-driven cycle:

1.  **Receive Command:** It accepts a target pose as either end-effector coordinates (XYZ + RPY) or direct joint angles.
2.  **Calculate Solution:** If given an end-effector pose, it uses the `roboticstoolbox` library to perform inverse kinematics (`ikine_LM`) and calculate the necessary joint angles (`q`).
3.  **Execute Movement:** It generates a `JointTrajectory` message with the target joint angles and a specified duration, publishing it to the `/scaled_joint_trajectory_controller` for the robot driver to execute.
4.  **Monitor State:** The node continuously subscribes to `/joint_states` to monitor the robot's real-time position.
5.  **Confirm Arrival:** It compares the current joint angles to the target angles. When the error is within a defined tolerance, it considers the movement complete.
6.  **Signal Readiness:** Upon completion, it publishes a "ready" signal to the `/<robot_name>/ready` topic, informing other nodes that it is ready to accept a new command.

### ROS API

#### Subscribed Topics
* **`/<robot_name>/point_pose`** (geometry_msgs/msg/Twist)
    * Receives a target end-effector pose (XYZ in mm, RPY in degrees). The node calculates the required joint angles using inverse kinematics.
* **`/<robot_name>/target_joint_pose`** (std_msgs/msg/Float32MultiArray)
    * Receives a direct command for target joint angles in degrees, bypassing the need for inverse kinematics.
* **`/<robot_name>/movement_duration`** (std_msgs/msg/Float32)
    * Receives the desired duration in seconds for the robot to move from its current position to the target.
* **`/joint_states`** (sensor_msgs/msg/JointState)
    * Receives the real-time joint angles from the robot hardware driver. This is used for state tracking and arrival confirmation.

#### Published Topics
* **`/scaled_joint_trajectory_controller/joint_trajectory`** (trajectory_msgs/msg/JointTrajectory)
    * Publishes the final trajectory command to the UR robot driver to execute the physical movement.
* **`/<robot_name>/ready`** (std_msgs/msg/Int8)
    * Publishes a status signal (`1`) when the robot has successfully reached its target pose and is ready for a new command. This is critical for synchronizing automated sequences.
* **`/<robot_name>/calculated_xyz`** (std_msgs/msg/Float32MultiArray)
    * Publishes the robot's current end-effector XYZ position, calculated in real-time using forward kinematics from the `/joint_states` data.
* **`/<robot_name>/joint_goal`** (std_msgs/msg/Float32MultiArray)
    * Publishes the target joint angles (`q`) that were calculated or received for the current movement.
* **`/<robot_name>/robot_log`** (std_msgs/msg/String)
    * Continuously publishes a formatted string containing logging information like the robot model, current XYZ position, and last received duration.
