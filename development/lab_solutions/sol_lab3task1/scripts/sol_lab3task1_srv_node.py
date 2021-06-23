#!/usr/bin/env python

import rospy
import numpy as np
import math
##TODO: Include the header file rotate_point service (look in the package example_msg_srv)
from sol_example_msg_srv.srv import rotate_point
from sol_example_msg_srv.srv import rotate_pointResponse
import pdb

##TODO: Complete this function
def rotate_point_3d(req):

	q0=req.q.w
	q1=req.q.x
	q2=req.q.y
	q3=req.q.z
	q1s=math.pow(q1,2)
	q2s=math.pow(q2,2)
	q3s=math.pow(q3,2)

	p1=req.input_p.x
	p2=req.input_p.y
	p3=req.input_p.z

	p1rot=(1-2*q2s-2*q3s)*p1 + 2*(q1*q2+q0*q3)*p2 + 2*(q1*q3-q0*q2)*p3
	p2rot=2*(q1*q2-q0*q3)*p1 + (1-2*q1s-2*q3s)*p2 + 2*(q2*q3+q0*q1)*p3
	p3rot=2*(q1*q3+q0*q2)*p1 + 2*(q2*q3-q0*q1)*p2 + (1-2*q1s-2*q2s)*p3


	res=rotate_pointResponse()
	res.output_p.x=p1rot
	res.output_p.y=p2rot
	res.output_p.z=p3rot
	return res

def rotate_point_server():
	rospy.init_node('rotate_point_server')
	##TODO: Intialise the service 'rotate_point' and have it calls the function 'rotate_point_3d'
	s = rospy.Service('rotate_point', rotate_point, rotate_point_3d)
	rospy.spin()

if __name__ == "__main__":
	rotate_point_server()
