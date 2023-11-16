# Coursework 2 Template
This stack contains the code template for achieving the second coursework. Students must fill in their code to this template and submit the whole stack as part of the coursework submission.

## CW2 - Question 4
`cw2q4` is a package for question 4 in the second coursework. Student should only fill the `youbotKineStudent.py` code template.

In the cw2q4 folder you can find three files.
* `youbotKineStudent.py`: This is the coding template that needs to be filled to answer question 4.
* `youbotKineBase.py`: This class includes common methods you may need to call in order to solve the questions below. **You should not edit this file.**
* `youbotKineKDL.py`: This class provides implementations to the questions below in KDL in order to check your own solutions. **You should not edit this file.**

### Task description 

a. Write a script to compute the Jacobian matrix for the YouBot manipulator. **[report - 2 pts, code - 10 pts]**

c. Write a script to detect singularity in any input pose. **[report - 1 pts, code - 4 pts]**


## CW2 - Question 6

`cw2q6` is a package for question 4 in the second coursework. 

### Task description 
Complete the following question by filling in the `cw2q6/cw2q6 node.py` python code templates to perform path planning via the shortest path. A simple code breakdown in the report is required for all subquestions, except subquestion e which is code only. You are given target joint positions in the bagfile `data.bag`. Your task is the Youbot end-effector to reach each target Cartesian check-point associated with each target joint position, via shortest path in Cartesian space. To solve the question you need to:

a. Implement the ”load targets()” method that loads the target joint positions from the bagfile and calculates the target end-effector position. **[report - 2 pts, code - 3 pts]**

b. Implement the ”get shortest path()” method that takes the checkpoint transformations and computes the order of checkpoints that results in the shortest overall path. **[report - 3 pts, code - 5 pts]**

c. Implement the ”decoupled rot and trans()” and ”intermediate tfs()” methods that take the target checkpoint transforms and the desired order based on the shortest path sorting, and create intermediate transformations by decoupling rotation and translation. **[report - 2 pts, code - 5 pts]**

d. Implement the ”ik position only()” and ”full checkpoints to joints()” methods that take the full set of checkpoint transformations, including intermediate checkpoints, and compute the associated joint positions with position only inverse kinematics. **[report - 2 pts, code - 8 pts]**

e. Implement the ”q6()” method, the main method of this question, where other methods are called in order to perform the path planning task. **[code - 5 pts]**
