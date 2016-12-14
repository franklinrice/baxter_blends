#!/usr/bin/env python
import sys, time, cv2, copy, tf
import rospy
import moveit_commander
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped, Pose
from baxter_interface import gripper as baxter_gripper
import subprocess
from sensor_msgs.msg import Image
#from lab4_cam.srv import ImageSrv, ImageSrvResponse
from cv_bridge import CvBridge, CvBridgeError

food_to_tag = {'apple': 'ar_marker_9', 'banana': 'ar_marker_9', 'orange':'ar_marker_9'}

cmd1 = ""

class BlenderBaxter:
	def setup(self):
		rospy.init_node("smoothie")
		### initialize grippers
		#rospy.init_node('gripper_test')
		self.right_gripper = baxter_gripper.Gripper('right')
		self.right_gripper.calibrate()
		### put blender and table into MoveIt constraints (might be done in RViz)
		
		### establish waypoints (dump, neutral)
		
		### create publisher for sending messages to Zumy
		
		### create subscriber for receiving messages from Zumy

		### create subscriber for receiving video from the hand cam
		#TESTING 
		# rospy.Subscriber("/usb_cam/image_raw", Image, self.imgReceived)
		#ON BAXTER 
		#input("make sure to run the image view and camera_control then press enter")
		#rospy.Subscriber("/cameras/right_hand_camera/image", Image, self.imgReceived)

		#rospy.init_node('ar_tags_subs')
		self.listener = tf.TransformListener()
		
		### initialize moveit
		#Initialize moveit_commander
		moveit_commander.roscpp_initialize([])#sys.argv)

		#Start a node
		# rospy.init_node('moveit_node')

		#Initialize both arms
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		#self.scene.addBox("table",.5, .5, .5, .35, -.4, -.6 )

		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = 0.5
		p.pose.position.y = 0.5
		p.pose.position.z = 0.5
		#self.scene.add_box("table", p, (0.35, -.4, -0.6))

		self.right_arm = moveit_commander.MoveGroupCommander('right_arm')
		self.right_arm.set_planner_id('RRTConnectkConfigDefault')
		self.right_arm.set_planning_time(8)
		#self.left_arm.set_planner_id('RRTConnectkConfigDefault')
		#self.left_arm.set_planning_time(10)
		self.fruit_1 = PoseStamped()
		self.fruit_1.header.frame_id = "base"

		x = input("move to table and enter aribtrary value")
		self.minZ = self.currentPos()[0][2]

		x = input("move to blender dump position")
		self.dump_pos, self.dump_rot = self.currentPos()
		print(self.dump_pos, self.dump_rot)

		x = input("move to dumped position")
		self.dumped_pos, self.dumped_rot = self.currentPos()

		x = input("move to neutral position")
		self.neutral_pos, self.neutral_rot = self.currentPos()

		#self.calibrate_to_blender();


	def main(self,args):
		#moveit_neutral()
		### open hand cam

		# self.set_waypoints()
		# sys.exit()


		### detect AR tag location (either directly or transformation)
		fruits = sys.argv[1:]
		while len(fruits) > 0:
			fruit = fruits.pop(0)

			# for fruit in args:
			if not fruit in food_to_tag.keys():
				print("We ran out of that fruit")
				sys.exit()

			dest_tag = food_to_tag[fruit]

			# ar_tags = {}
			# ar_tags['ar0'] = 'ar_marker_' + sys.argv[1]
			# ar_tags['ar1'] = 'ar_marker_' + sys.argv[2]
			# ar_tags['arZ'] = 'ar_marker_' + sys.argv[3]
			# check launch file to make sure moveit is relative to base
			self.listener.waitForTransform('/base',dest_tag, rospy.Time(0), rospy.Duration(4))
			(fruit_trans, fruit_rot) = self.listener.lookupTransform('/base',dest_tag,rospy.Time(0))
			print(fruit_trans)

			### MoveIt to that position

			self.move_to_pose_from_transform(fruit_trans, True)
			self.grab()

			self.move_to_pose_from_transform(self.neutral_pos, False)

			self.move_to_pose_from_transform(self.dump_pos, False)
			self.rotate_to_pose_from_transform(self.dumped_rot)
			self.release()

			self.move_to_pose_from_transform(self.neutral_pos, False)

			#self.move_to_full_pose(self.dump_pos, self.dump_rot)
			#self.move_to_full_pose(self.neutral_pos, self.neutral_rot)

		# We wil likely have to do a bit of fine-tuning here, so we grab in the right spot. Consider using the range sensor
		### make sure in exactly the right position before grabbing, then grab
		
		#left_gripper.close(block=True)

		### grab, Off). If the component is an output, then the state field will be the current setting of the output. isInputOnly: field tells you if the component is an input (sensor) only, and cannot ou
		
		### move to above blender waypoint
		#moveit_blender()
		### dump (rotate)
		
		### return to neutral waypoint
		#moveit_fruit()
		# left_gripper.open(block=True)
		### activate blender?

	def currentPos(self):
		(trans,rot) = self.listener.lookupTransform('/base','/right_gripper',rospy.Time(0))
		return trans, rot

	

	def grab(self):
		self.right_gripper.close(block=True)
		rospy.sleep(.2)
		self.right_gripper.close(block=True)
		rospy.sleep(.5)

	def release(self):
		self.right_gripper.open(block=True)
		rospy.sleep(1.0)

	def move_to_pose_from_transform(self, trans, onTable=False):
		dest = PoseStamped()
		dest.header.frame_id = 'base'
		dest.pose.position.x = trans[0] 
		dest.pose.position.y = trans[1] 
		dest.pose.position.z = trans[2] 
		if onTable:
			dest.pose.orientation.x += 1
			dest.pose.position.z = self.minZ
		
		#Orientation as a quaternion
		dest.pose.orientation.x = 0
		dest.pose.orientation.y = -1
		dest.pose.orientation.z = 0
		dest.pose.orientation.w = 0

		#Set the goal state to the pose you just defined
		self.right_arm.set_pose_target(dest)
		#Set the start state for the left arm
		self.right_arm.set_start_state_to_current_state()
		
		#Create a path constraint for the arm
		orien_const = OrientationConstraint()
		orien_const.link_name = "right_gripper";
		orien_const.header.frame_id = "base";
		orien_const.orientation.y = -1.0;
		### Make sure to set these back to .1
		orien_const.absolute_x_axis_tolerance = .1;
		orien_const.absolute_y_axis_tolerance = .1;
		orien_const.absolute_z_axis_tolerance = .1;
		orien_const.weight = 1.0;
		consts = Constraints()
		consts.orientation_constraints = [orien_const]
		#self.left_arm.set_path_constraints(consts)

		#Plan a path
		right_plan = self.right_arm.plan()

		#Execute the plan
		self.right_arm.execute(right_plan)

	def move_to_full_pose(self, trans, rot, onTable=False):
		dest = PoseStamped()
		dest.header.frame_id = 'base'
		dest.pose.position.x = trans[0] 
		dest.pose.position.y = trans[1] 
		dest.pose.position.z = trans[2] 
		if onTable:
			dest.pose.position.z = self.minZ
		
		#Orientation as a quaternion
		dest.pose.orientation.x = rot[0]
		dest.pose.orientation.y = rot[1]
		dest.pose.orientation.z = rot[2]
		dest.pose.orientation.w = 0

		#Set the goal state to the pose you just defined
		self.right_arm.set_pose_target(dest)
		#Set the start state for the left arm
		self.right_arm.set_start_state_to_current_state()
		
		#Create a path constraint for the arm
		orien_const = OrientationConstraint()
		orien_const.link_name = "right_gripper"
		orien_const.header.frame_id = "base"
		orien_const.orientation.x = rot[0]
		orien_const.orientation.y = rot[1]
		orien_const.orientation.z = rot[2]
		### Make sure to set these back to .1
		orien_const.absolute_x_axis_tolerance = .1
		orien_const.absolute_y_axis_tolerance = .1
		orien_const.absolute_z_axis_tolerance = .1
		orien_const.weight = 1.0
		consts = Constraints()
		consts.orientation_constraints = [orien_const]
		#self.left_arm.set_path_constraints(consts)

		#Plan a path
		right_plan = self.right_arm.plan()

		#Execute the plan
		self.right_arm.execute(right_plan)

	def moveit_fruit(self,trans):
		fruit = PoseStamped()
		fruit.header.frame_id = 'base'
		fruit.pose.position.x = trans[0] + .1 #from AR Tag
		fruit.pose.position.y = trans[1] #from AR tag
		fruit.pose.position.z = self.minZ #trans[2] +.05#from AR tag
		# fruit.pose.position = trans
		
		#Orientation as a quaternion
		fruit.pose.orientation.x = 0#from AR tag
		fruit.pose.orientation.y = -1#from AR tag
		fruit.pose.orientation.z = 0#from AR #tag
		fruit.pose.orientation.w = 0#from AR tag
		# fruit.pose.orientation = q
		
		#Set the goal state to the pose you just defined
		self.right_arm.set_pose_target(fruit)
		#Set the start state for the left arm
		self.right_arm.set_start_state_to_current_state()
		
		#Create a path constraint for the arm
		orien_const = OrientationConstraint()
		orien_const.link_name = "right_gripper"
		orien_const.header.frame_id = "base";
		orien_const.orientation.y = -1.0;
		### Make sure to set these back to .1
		orien_const.absolute_x_axis_tolerance = .1;
		orien_const.absolute_y_axis_tolerance = .1;
		orien_const.absolute_z_axis_tolerance = .1;
		orien_const.weight = 1.0;
		consts = Constraints()
		consts.orientation_constraints = [orien_const]
		self.right_arm.set_path_constraints(consts)

		#Plan a path
		right_plan = self.right_arm.plan()

		#Execute the plan
		self.right_arm.execute(right_plan)
		
	def set_pose_stamped(pose_stamped, x_position, y_position, z_position, x_orientation, y_orientation, z_orientation, w_orientation, header_frame_id = "base"):
		# PoseStamped is a geometry_msg with a header and a Pose
		# Pose is a geometry_msg with a position.x,y,z and orientation.x,y,z,w
		# the rest are float64

		# delet these l8r
		# Seems like we only used header to make header.frame_id = "base"... personally not sure why

		pose_stamped.header.frame_id = header_frame_id
		pose_stamped.pose.position.x = x_position
		pose_stamped.pose.position.y = y_position
		pose_stamped.pose.position.z = z_position
		pose_stamped.pose.orientation.x = x_orientation
		pose_stamped.pose.orientation.y = y_orientation
		pose_stamped.pose.orientation.z = z_orientation
		pose_stamped.pose.orientation.w = w_orientation

		return pose_stamped
		
	def make_constraint(orien_const, link_name, x_orientation, y_orientation, z_orientation, w_orientation, x_tolerance=.1, y_tolerance=.1, z_tolerance=.1, weight = 1.0, frame_id = "base"):
		# orien_const is of type moveit_msgs/OrientationConstraint
		# link_name should be the name what is being constrained, e.g. "left_gripper"
		# the orientations and tolerances should all be float64
		# the weight gives importance of a constraint relative to other constraints, 0 means less important

		orien_const.link_name = link_name
		orien_const.header.frame_id = frame_id
		orien_const.orientation.x = x_orientation
		orien_const.orientation.y = y_orientation
		orien_const.orientation.z = z_orientation
		orien_const.orientation.w = w_orientation
		orien_const.absolute_x_axis_tolerance = x_tolerance
		orien_const.absolute_y_axis_tolerance = y_tolerance
		orien_const.absolute_z_axis_tolerance = z_tolerance
		orien_const.weight = weight

		return orien_const
		
		
	def move_to_goal(self,goal, limb, constraints):
		# goal is of type PoseStamped
		# constraints is an array of constraints
		# plans and moves limb to the goal state

		# delet these comments l8r
		# just a wrapper for stuff we wrote in lab7

		limb.set_pose_target(goal)
		limb.set_start_state_to_current_state()
		# constraints stuff
		consts = Constraints()
		consts.orientation_constraints = constraints
		if len(consts) > constraints:
			self.right_arm.set_path_constraints(consts)

		plan = limb.plan()
		# could do some path checking
		limb.execute(plan)

	# def imgReceived(self, message):
	# 	x = 5
	# 	#do lots of stuff

	# def calibrate_to_blender(self):
	# 	input("Move baxter to the dumping point and press enter")
	# 	#self.dump_point = self.left_arm.get_current_pose().pose
	# 	self.dump_point = self.currentPos()
	# 	print(self.dump_point)
		
	# def moveit_neutral():
	# 	# initial position -------------------------------------------------------
	# 	initial.pose.position.x = #initial position
	# 	initial.pose.position.y = #initial position
	# 	initial.pose.position.z = #initial position
		
	# 	#Orientation as a quaternion
	# 	initial.pose.orientation.x = #initial position
	# 	initial.pose.orientation.y = #initial position
	# 	initial.pose.orientation.z = #initial position
	# 	initial.pose.orientation.w = #initial position
		
	# 	#Set the goal state to the pose you just defined
	# 	self.left_arm.set_pose_target(initial)
	# 	#Set the start state for the left arm
	# 	self.left_arm.set_start_state_to_current_state()
		
	# 	#construct table
	# 	collision_object = moveit_msgs.msg.CollisionObject()

	# 	#Plan a path
	# 	left_plan = self.left_arm.plan()

	# 	#Execute the plan
	# 	self.left_arm.execute(left_plan)
	 
	

	# def calibrate_blender_position(self):
	# 	input("move baxter arm to dump position then press enter")
	# 	self.blenderPos = self.left_arm.get_current_pose()

	# def set_waypoints(self):
	# 	waypoints = []

	# 	(trans,rot) = self.listener.lookupTransform('/base','/left_gripper',rospy.Time(0))

	# 	# start with the current pose
	# 	no = self.left_arm.get_current_pose().pose

	# 	#waypoints.append()


	# 	# first orient gripper and move forward (+x)
	# 	wpose = Pose()
	# 	wpose.position.x = trans[0]
	# 	wpose.position.y = trans[1]
	# 	wpose.position.z = trans[2]
	# 	wpose.orientation.w = 1.0
	# 	wpose.orientation.x = rot[0]
	# 	wpose.orientation.y = rot[1]
	# 	wpose.orientation.z = rot[2]
	# 	# wpose.position.x = waypoints[0].position.x + 0.1
	# 	# wpose.position.y = waypoints[0].position.y
	# 	# wpose.position.z = waypoints[0].position.z
	# 	waypoints.append(copy.deepcopy(wpose))

	# 	# second move down
	# 	wpose.position.z -= 0.10
	# 	waypoints.append(copy.deepcopy(wpose))

	# 	wpose.position.z -= 0.10
	# 	waypoints.append(copy.deepcopy(wpose))
	# 	wpose.position.z -= 0.10
	# 	waypoints.append(copy.deepcopy(wpose))
	# 	wpose.position.z -= 0.10
	# 	waypoints.append(copy.deepcopy(wpose))

	# 	# third move to the side
	# 	wpose.position.y += 0.05
	# 	waypoints.append(copy.deepcopy(wpose))



	# 	(plan, fraction) = self.left_arm.compute_cartesian_path(
	# 	                             waypoints,   # waypoints to follow
	# 	                             0.01,        # eef_step
	# 	                             0.0)         # jump_threshold

	# 	self.left_arm.execute(plan)

	# def moveit_blender():
		
	# 	blender.pose.position.x = #from AR Tag
	# 	blender.pose.position.y = #from AR tag
	# 	blender.pose.position.z = #from AR tag
		
	# 	#Orientation as a quaternion
	# 	blender.pose.orientation.x = #from AR tag
	# 	blender.pose.orientation.y = #from AR tag
	# 	blender.pose.orientation.z = #from AR tag
	# 	blender.pose.orientation.w = #from AR tag
		
	# 	#Set the goal state to the pose you just defined
	# 	left_arm.set_pose_target(self.blenderPos)
	# 	#Set the start state for the left arm
	# 	left_arm.set_start_state_to_current_state()
		
	# 	#Create a path constraint for the arm
	# 	orien_const = OrientationConstraint()
	# 	orien_const.link_name = "left_gripper";
	# 	orien_const.header.frame_id = "base";
	# 	orien_const.orientation.y = -1.0;
	# 	orien_const.absolute_x_axis_tolerance = 0.1;
	# 	orien_const.absolute_y_axis_tolerance = 0.1;
	# 	orien_const.absolute_z_axis_tolerance = 0.1;
	# 	orien_const.weight = 1.0;
	# 	consts = Constraints()
	# 	consts.orientation_constraints = [orien_const]
	# 	left_arm.set_path_constraints(consts)

	# 	#Plan a path
	# 	left_plan = left_arm.plan()

	# 	#Execute the plan
	# 	left_arm.execute(left_plan)

if __name__ == '__main__':
	bax = BlenderBaxter()
	bax.setup()
	bax.main(sys.argv)