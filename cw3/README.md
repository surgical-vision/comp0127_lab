# coursework 3 template
This stack contains the code template for achieving the third coursework. Students must fill in their code to this template and submit the whole stack as part of the coursework submission.

cw3q2 is a package for question 2 in this coursework. This package's structure is very similar to the one in "kdl_kine" package, i.e. it does not build any executable, but rather create a library which the other packages "depend on". Keep in mind that you have to manually implement the computation by yourself, not using the kdl library. The kdl library in "kdl_kine" package is for checking whether your Jacobian and inverse kinematic solution is correct. The examples on how to run the kdl library is shown in lab09example01. You may need to build your own checking node that calls both "kdl_kine" function and "cw3q2" function to make sure your solution is correct, but this node is not a part of the assessment. 

cw3q5 is a package for the last question in this coursework. 

For 5a, you should start by loading the bag specified in the pdf. You can compute the acceleration by using the data read from JointState after the trajectory is executed.

For 5b, you have to move the robot into three configurations and calculate mass and the centre of mass with respect to the last frame and print the values out.

Please note that you have to edit your launch files in cw3_launch to run your node. 
