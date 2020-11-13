# coursework 2 template
This stack contains the code template for achieving the second coursework. Students must fill in their code to this template and submit the whole stack as part of the coursework submission.

cw2q4 is a package for question 4 in the second coursework. This package's structure is very similar to the one in "kdl_kine" package, i.e. it does not build any executable, but rather create a library which the other packages "depend on". The code written in cw2q4 should be able to compute Jacobian, iterative inverse kinematics and check for singularity. Keep in mind that you have to manually implement the computation by yourself, not using the kdl library. The kdl library in "kdl_kine" package is for checking whether your Jacobian and inverse kinematic solution is correct. The examples on how to run the kdl library is shown in lab05example01. You may need to build your own checking node that calls both "kdl_kine" function and "cw2q4" function to make sure your solution is correct, but this node is not a part of the assessment. 

cw2q6 is a package for the last question in the second coursework. First, you node has to load a specific bag by giving the input into the "question" option defined in the launch file. Then, your node will have to come up with a trajectory that satisfies criteria in each question. 

For 6a, the shortest path attempt should be done by a linear trajectory between any two points, but there are challenges you will have to solve later. However, for only this question, if we do not see any attempt on optimising the path, i.e. you only use JointTrajectory message to move your arm, your coding part will not be marked. 

For 6b, your robot arm only needs to reach all the checkpoint simulated in rviz without crashing into any obstacle. There are so many solutions to the problem. You could implement the potential field method, Probabilistic roadmap method, you could look up some state-of-the-art path planning method and apply here or you could devise a strategy to move the robot around the environment without clashing. Whatever method you choose, please explain it well in your report and please do not hard-code the trajectory in your script otherwise your marks will be deducted by half. 

An example of hard-coding your trajectory would be like this,
- You know that you have to move the end-effector from point A to point B, but the path clashes into an obstacle.
- You create sub-trajectory to move around to point C and D by MANUALLY inputing these two points, knowing that the new path (A-->C-->D-->B) will no longer clash into any obstacle.
- As a result, you command the end-effector from point A to point B without clashing. This will result in your mark for the coding part being deducted by half.

What is considered as a strategy would be,
- You know that you have to move the end-effector from point A to point B, but the path clashes into an obstacle.
- You create sub-trajectory to move around to point C and D by COMING UP with a formula to compute these two points, knowing that the new path (A-->C-->D-->B) will no longer clash into any obstacle.
- As a result, you command the end-effector from point A to point B without clashing. This will not result in any deduction.
