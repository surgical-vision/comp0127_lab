# Coursework 3 Template
This stack contains the code template for achieving the second coursework. Students must fill in their code to this template and submit the whole stack as part of the coursework submission.

Complete question 2 by filling in the ```cw3q2/iiwa14DynStudent.py``` python
class code template, to compute the dynamic components for the KUKA LBR iiwa14
manipulator.

In the cw3q2 folder, you can find three files:
- ```iiwa14DynStudent.py```: This is the class template for the questions below.
- ```iiwa14DynBase.py```: This class includes common methods you may need to call in order to solve question 2. You should not edit this file.
- ```iiwa14DynKDL.py```: This class provides implementations to the questions in KDL which can be used to check your own solutions. You should not edit this file.

In question 5, you are tasked with computing the joint accelerations throughout the trajectory defined in the bagfile ```cw3q5/bag/cw3q5.bag``` using dynamic components. You should also plot the computed joint accelerations. You only need to edit the ```cw3q5.py``` file, and the corresponding launch file. Note that a coding template is not provided for this question. For the dynamic components, you can use either your own implementation from question 2, or the corresponding KDL class.