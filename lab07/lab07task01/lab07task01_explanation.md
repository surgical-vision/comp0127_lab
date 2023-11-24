
# Lab07 Task01: Youbot Trajectory Control and Visualization

This ReadMe explains the Lab07 Task01 solution. In this code, you are asked to control a Youbot, by specifying joint trajectories and visualizing them.

## Code Explanation

### 1. Loading Joint Targets into a Numpy Array
```python
for i in range(joint_targets.shape[1]):
    joint_targets[:, i] = hardcoded_joint_targets[i]
```
This segment iterates over the columns of the `joint_targets` numpy array (initially all zeros) and fills them with values from `hardcoded_joint_targets`. This transfers predefined joint positions into the `joint_targets` array for further processing.

### 2. Computing Forward Kinematics and Storing the Results
```python
checkpoint = np.zeros([4, 4, joint_targets.shape[1]])
for i in range(joint_targets.shape[1]):
    checkpoint[:, :, i] = kdl_youbot.forward_kinematics(list(joint_targets[:, i]))
```
Here, the code computes the forward kinematics for each set of joint angles in `joint_targets`. It initializes a 3-dimensional numpy array `checkpoint` to store transformation matrices. For each joint angle set, it calls `forward_kinematics` of `kdl_youbot` and stores the resulting 4x4 transformation matrix in `checkpoint`.

### 3. Publishing the Joint Trajectory
```python
t = 10
dt = 2
for i in range(joint_targets.shape[1]):
    traj_point = JointTrajectoryPoint()
    traj_point.positions = joint_targets[:, i]
    t = t + dt
    traj_point.time_from_start.secs = t
    traj.points.append(traj_point)

traj.header.stamp = rospy.Time.now()
traj.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
traj_pub.publish(traj)
```
This section creates a joint trajectory message (`traj`) that the robot will follow. It iterates over `joint_targets`, creating a `JointTrajectoryPoint` for each target, setting positions, and incrementing time (`t`) by a delta time (`dt`). These points are appended to `traj`. After setting joint names and timestamp, the trajectory message is published to `traj_pub`.

### 4. Publishing Checkpoints
The `publish_checkpoints` function takes a 3D array of transformation matrices (`tfs`) and publishes them as visual markers, each representing a point in the Cartesian path of the robot's end-effector. The markers are color-coded to show the sequence.

## Main Function
The main function (`youbot_traj`) initializes the ROS node, creates publishers for trajectory and checkpoints, prepares joint targets, computes their forward kinematics, and publishes the trajectory for the robot to follow. The `publish_checkpoints` function is presumably called within the commented-out section of the main function to visualize the computed checkpoints.
