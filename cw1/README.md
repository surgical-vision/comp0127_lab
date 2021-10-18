# coursework 1 template
This stack contains the code template for achieving the first coursework. Students must fill in code templates and submit the whole stack as part of the coursework submission.

cw1q4 is a package for question 4 in the first coursework. Students should write a code to create two "ROS service" to convert rotation representations as stated in the instruction. The services are defined in cw1q4_srv.

cw1q4_srv contains the definition of the services used in cw1q4 package. Students should change the parameters in the two definitions as appropriate. Please do not change the name of the three services as this will invoke an error.

cw1q5b and cw1q5d contain the code templates for question 5b and question 5d, respectively. The work in both packages are very similar to the examples shown during the lab sessions. The only difference is they are based on different models of the KUKA youbot manipulator. There are three things to keep in mind when working on these questions,
1. Even if your code in cw1q5b is perfect, the frames you define will not align perfectly with the rviz model because the DH parameters are based on the simplified version. This discrepancy will not affect your marks in any way.
2. cw1q5c and cw1q5d require you to read the xacro file. It is basically a robot model for simulation, defining where robot parts, frames, links ond joints are in the model. This question may take you some time to work it out, but it is expected as it is the hardest question of this coursework.
3. The joint positions in the hardware interface and the ones that result from forward kinematics are usually similar and most of the time you are not required to account for offsets. However, this is not the case with the youbot manipulator. Please take a look at the "origin, rpy" and "limit" in the xacro file noted in question 5c and work out how to change the joint inputs accordingly. Without any modification, your robot arm may end up moving in the opposite direction or have an offset in the joint position you have not accounted for.
