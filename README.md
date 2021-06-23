# comp0127_lab
Collection of packages for the lab sessions in COMP0127 Robotic Systems Engineering

development folder:
- Main area of lab and coursework solutions and work in progress.
- Once finalized, can be moved to release folder or the version the students recieve.

release folder:
- Student version of code for lab and coursework.
- During course, zip appropriate folder for coursework and post on moodle.

Dependencies:
`sudo apt install ros-melodic-controller-manager ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-gazebo-ros-control ros-melodic-joint-trajectory-controller ros-melodic-velocity-controllers ros-melodic-ros-controllers ros-melodic-ros-control`

Note:
In case you use the ROS distribution rather than kinetic, e.g. noetic, melodic, etc, you can change `melodic` in the command to match your distribution accordingly. Please also note that your ROS distribution has to be compatible with your Ubuntu version.
`sudo apt install ros-<distro>-controller-manager ros-<distro>-joint-state-controller ros-<distro>-effort-controllers ros-<distro>-gazebo-ros-control ros-<distro>-joint-trajectory-controller ros-<distro>-velocity-controllers ros-<distro>-ros-controllers`

References:
- youbot_description 'https://github.com/youbot'
- manipulator_h_description 'https://github.com/ROBOTIS-GIT/ROBOTIS-MANIPULATOR-H'
- open_manipulator_description 'https://github.com/ROBOTIS-GIT/open_manipulator'
