# Coursework 2 Template
This stack contains the code template for achieving the second coursework. Students must fill in their code to this template and submit the whole stack as part of the coursework submission.

### CW2Q4 Template
For this question, students must fill out the template in the cw2q4 package. It has been implemented with a youbot kinematic
base class, which implements some common functions. The student template inheiris from this base class, with the functions
that students must fill out. Also given is a KDL implementation for later questions. Lastly, a reference solution is given that
is a filled out version of the student template.

#### CW2Q4 Testing
To test student solutions, under cw2q4/tests/tester_cw2_q4.py, there are some functions to test various implementations.
* The first test is testing the Jacobian. For this test, 5 joints are used and the Jacobian of the student solution vs.
the reference solution are compared. For each set of joint values, a mark out of 10 is given in the test.
  
* The second test is testing the iterative inverse kinematics. The test takes the same set of joints from the Jacobian test
and generates desired poses using the reference forward kinematics solution. The student interative solution is then tested by
  setting the initial joint position to zeros and setting the desired pose. Then the error between the final student IK solution 
  and desired pose is compared.
  
* The final test is of the singularity solution. The test is comparing the reference singularity test to that of the student.

To run the test for CWQ4 run the following command.
```rostest cw2q4 cw2q4.test```

#### CW2Q6 Template
