#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage	# lab6
from geometry_msgs.msg import Transform, Vector3
import kin_func_skeleton as kfs
import exp_quat_func as eqf
import baxter_blend as bb
import rospy	# lab7_baxter
from baxter_interface import gripper as baxter_gripper 
import rospy	# lab3
from sensor_msgs.msg import JointState
import fk

lkp = {} # dictionary containing the last known position of AR tags
gdef = np.ones((4,4)) #[[0,0,0,0],[0,0,0,0],[0,0,0,0], [0,0,0,0]]


def baxter_fruit_to_blender_callback(msg, ar_tag):

	right_gripper = baxter_gripper.Gripper('right')	# lab7_baxter
	print('Calibrating...')
	left_gripper.calibrate()
	rospy.sleep(2.0)

	matrix = []
	def forward_kinematics_callback(js):	# lab3
	    #Print the contents of the message to the console
	    # print(js)
	    # print(js.position)
	    matrix = fk.final_matrix(js.position[4], js.position[5], js.position[2], js.position[3], js.position[6], js.position[7], js.position[8])
	    print("forward kinematics matrix:")
	    print(matrix)


	for i in range(len(msg.transforms)): # lab6
		trans = msg.transforms[i].transform.translation
		# make sure the final position is about a centimeter above the object
		trans.z = trans.z - 0.01
		# q = msg.transforms[i].transform.rotation
		q.x = 0.0
		q.y = 0.0
		q.z = -1.0
		q.w = 0.0

		# README
		# use lab3 and the forward kinematics to calculate the position of the hand
			# this gives you a matrix from the base frame to the hand frame
			# then find the matrix inverse and use to to go from hand frame to base frame
			# use this matrix to take the ar_tag position into the body frame
		
		# dont know if I need this section of code
		# once the position of the hand is found use lab6 and the create_rbt to find transform between hand and body frames
		# once the rigid body transform is found use it put the ar_tag the hand sees into the frame of the body
		# once you have that then reassign trans and q so they reflect the body position
		
		# then pass in the position into the moveit_fruit and moveit_blender
		# other ideas, hardcode in the positions of the blender
		# still need to include the zumy but will work on it
		# the callback function is only there for reference and will likely not be used.
		 
		rospy.Subscriber("/robot/joint_states", JointState, callback) # lab3

		right_gripper.open(block=True)	# lab7_baxter
		moveit_fruit(trans, q)
		right_gripper.close(block=True)
		moveit_blender()
		right_gripper.open(block=True)
		moveit_initial()

def callback(msg, ar_tag):
	#for i in range(0, len(msg.transforms)):
	#lkp[ar_tags['ar0']] = gdef
	#lkp[ar_tags['ar1']] = gdef
	#lkp[ar_tags['arZ']] = gdef
	for i in range(len(msg.transforms)):
		
		print(len(msg.transforms))

		trans = msg.transforms[i].transform.translation
		q = msg.transforms[i].transform.rotation
		omega, theta = eqf.quaternion_to_exp(q)
		# print(omega,theta,trans)
		trans = [trans.x, trans.y, trans.z]
		g = eqf.create_rbt(omega,theta,trans)

		tag = msg.transforms[i].child_frame_id
		lkp[tag] = g
		print(tag)

		# print(lkp.keys())
		# print(ar_tags)
	if len(lkp.keys()) < 3:
		print(tag)
		return
	gAR0 = lkp[ar_tags['ar0']]
	gAR1 = lkp[ar_tags['ar1']]
	gARZ = lkp[ar_tags['arZ']]
		# YOUR CODE HERE
		# The code should look at the transforms for each AR tag
		# Then compute the rigid body transform between AR0 and AR1, 
		# AR0 and ARZ, AR1 and ARZ
		#  hint: use the functions you wrote in exp_quat_func
		#  note: you can change anything in this function to get it working
		# #  note: you may want to save the last known position of the AR tag
		# trans0 = msg.transforms[0].transform.translation
		# trans1 = msg.transforms[1].transform.translation
		# transZ = msg.transforms[2].transform.translation

		# q0 = msg.transforms[0].transform.rotation
		# #q1 = msg.transforms[1].transform.rotation
		# #qz = msg.transforms[2].transform.rotation

		# omegaAR0, thetaAR0 = eqf.quaternion_to_exp(q0) # from quaternion to rotation
		# omegaAR1, thetaAR1 = eqf.quaternion_to_exp(q1)
		# omegaARZ, thetaARZ = eqf.quaternion_to_exp(q2)

		# gAR0 = eqf.create_rbt(omegaAR0,thetaAR0,trans0) #create individual g matrix
		# gAR1 = eqf.create_rbt(omegaAR1,thetaAR1,trans1)
		# gARZ = eqf.create_rbt(omegaARZ,thetaARZ,trans2)        



		# #trans01 = Vector3.AR0 - lkp['AR1'] # solve for trans 
		# #trans1Z = lkp['AR1'] - lkp['ARZ']
		# #trans0Z = lkp['AR0'] - lkp['ARZ']
		# print("$$$$$$$$$$$$$$$$$$$$$$$$$$$$$")
		# print(gAR0)
		# print(gAR1)
		# print(gARZ)

	g01 = eqf.compute_gab(gAR0,gAR1) #transformation matrix bt frames
	g1z = eqf.compute_gab(gAR1,gARZ)
	g0z = eqf.compute_gab(gAR0,gARZ)
	print("transform from 1 to 0")
	print(g01)
	print("transform from z to 1")
	print(g1z)
	print("transform from z to 0")
	print(g0z)
		
		
		#q = [q0,q1,q2]
		
		#for i in range(3):
		#    lkp[msg.transforms[i].child_frame_id] = q[i] # position / orientation

		# print msg.transforms[i]

if __name__=='__main__':
	rospy.init_node('ar_tags_subs_manual')

	# change this to register every ar_tag baxter sees
	if len(sys.argv) < 4:
		print('Use: ar_tags_subs_manual.py [ AR tag number ] [ AR tag number ] [ AR tag number ] ')
		sys.exit()
	ar_tags = {}
	ar_tags['ar0'] = 'ar_marker_' + sys.argv[1]
	ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
	ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]
	print(sys.argv)

	#lkp[ar_tags['ar0']] = gdef
	#lkp[ar_tags['ar1']] = gdef
	#lkp[ar_tags['arZ']] = gdef

	rospy.Subscriber('/tf', TFMessage, baxter_fruit_to_blender_callback, ar_tags)
	rospy.spin()
