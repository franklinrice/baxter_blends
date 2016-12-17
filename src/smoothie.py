#!/usr/bin/env python
import sys, time, cv2, copy, tf, rospy
import pickle
import moveit_commander
import numpy as np
from moveit_msgs.msg import OrientationConstraint, Constraints
from geometry_msgs.msg import PoseStamped, Pose
from baxter_interface import gripper as baxter_gripper
import subprocess
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

food_to_tag = {'orange': 'ar_marker_14', 'apple': 'ar_marker_12', 'banana': 'ar_marker_16', 'banana2':'ar_marker_5', 'giraffe': 'ar_marker_8'}
zumy_tag = 'ar_marker_9'

class BlenderBaxter:
	def setup(self):
		rospy.init_node("smoothie")
		### initialize grippers
		self.gripper = baxter_gripper.Gripper('left')
		self.gripper.calibrate()
		
		### 
		self.listener = tf.TransformListener()
		
		### Initialize moveit
		moveit_commander.roscpp_initialize([])

		#Initialize desired arm
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()

		p = PoseStamped()
		p.header.frame_id = self.robot.get_planning_frame()
		p.pose.position.x = 0.5
		p.pose.position.y = 0.5
		p.pose.position.z = 0.5
		#self.scene.add_box("table", p, (0.35, -.4, -0.6))

		self.main_arm = moveit_commander.MoveGroupCommander('left_arm')
		self.main_arm.set_planner_id('RRTConnectkConfigDefault')
		self.main_arm.set_planning_time(10)
		# self.secondary_arm = moveit_commander.MoveGroupCommander('right_arm')
		# self.secondary_arm.set_planner_id('RRTConnectkConfigDefault')
		# self.secondary_arm.set_planning_time(10)
		self.fruit_1 = PoseStamped()
		self.fruit_1.header.frame_id = "base"

		if sys.argv[-1] == "load":
			with open('waypoints.pkl', 'rb') as input1:
			    self.dump_pos = pickle.load(input1)
			    # self.dumped_rot = pickle.load(input1)
			    self.halfway_pos = pickle.load(input1)
			    self.neutral_pos = pickle.load(input1)
			    self.minZ = pickle.load(input1)
			    self.action_pos = pickle.load(input1)
		else:
			x = input("move to table and enter aribtrary value")
			self.minZ = self.currentPos()[0][2]

			x = input("move to blender dump position")
			self.dump_pos, self.dump_rot = self.currentPos()
			print(self.dump_pos, self.dump_rot)

			# x = input("move to dumped position")
			# self.dumped_pos, self.dumped_rot = self.currentPos()

			x = input("press blender on button (1cm above)")
			self.action_pos, self.action_rot = self.secondaryPos()

			x = input("move halfway to neutral position")
			self.halfway_pos, self.halfway_rot = self.currentPos()
			# self.halfway_pos[2] += .3

			x = input("move to neutral position")
			self.neutral_pos, self.neutral_rot = self.currentPos()
			

			with open('waypoints.pkl', 'wb') as output:
				pickle.dump(self.dump_pos, output, -1)
				# pickle.dump(self.dumped_rot, output, -1)
				pickle.dump(self.halfway_pos, output, -1)
				pickle.dump(self.neutral_pos, output, -1)
				pickle.dump(self.minZ, output, -1)
				pickle.dump(self.action_pos, output, -1)


	def main(self,args):
		fruits = sys.argv[1:]
		while len(fruits) > 0:
			print(fruits)
			fruit = fruits.pop(0)

			if not fruit in food_to_tag.keys():
				if fruit == "load":
					break
				print("We ran out of that fruit")
				sys.exit()

			dest_tag = food_to_tag[fruit]

			print("getting " + fruit + " at " + dest_tag)
			self.listener.waitForTransform('/base',dest_tag, rospy.Time(0), rospy.Duration(10))
			try:
				(self.fruit_trans, self.fruit_rot) = self.listener.lookupTransform('/base',dest_tag,rospy.Time(0))
			except e:
				if fruit[-1] != "load":
					fruits.append(fruit)
				else: 
					load = fruit.pop(-1)
					fruits.append(fruit)
					fruits.append(load)
			print(self.fruit_trans)

			### MoveIt to that position
			self.move_to_fruit()
			self.grab()

			self.move_to_blender_through_neutral()
			self.dump()
			self.move_to_pose_from_transform(self.halfway_pos)
			self.move_to_pose_from_transform(self.neutral_pos)	
		
		self.get_zumy()

		### Uncomment this to actually make the smoothie (press power button)
		### you will have had to uncomment the lines above as well
		#print("about to make smoothie")
		#self.make_smoothie()

	def currentPos(self):
		(trans,rot) = self.listener.lookupTransform('/base','/left_gripper',rospy.Time(0))
		return trans, rot

	def secondaryPos(self):
		(trans,rot) = self.listener.lookupTransform('/base','/right_gripper',rospy.Time(0))
		return trans, rot

	def grab(self):
		self.gripper.close(block=True)
		rospy.sleep(.2)
		self.gripper.close(block=True)
		rospy.sleep(.5)

	def release(self):
		self.gripper.open(block=True)
		rospy.sleep(1.0)

	def dump(self):
		### This line is for simply releasing fruit
		self.release()

		### Uncomment the following lines (and comment the above) for cup dumping
		#curr = self.currentPos()[1]
		#self.rotate_to_pose_from_transform(self.dumped_rot)
		#rospy.sleep(2)
		#self.rotate_to_pose_from_transform(curr)

	def make_smoothie(self):
		print("about to turn on")
		ready = np.add(self.action_pos, [0,0,.02])
		on_pos = np.subtract(self.action_pos, [0,0,.02])
		self.move_to_pose_from_transform(ready)
		self.move_to_pose_from_transform(on_pos)

	def move_to_fruit(self):
		wp1 = np.add(self.fruit_trans, [.1,0,.15])
		wp2 = np.add(self.fruit_trans, [.1,0,.1])
		self.move_to_pose_from_transform(wp1, False)
		self.move_to_pose_from_transform(wp2, False)
		self.move_to_pose_from_transform(self.fruit_trans, True)

	def move_to_blender_through_neutral(self):
		print("going to neutral")
		wp1 = np.divide(np.sum([self.currentPos()[0], self.neutral_pos], axis=0),2)
		self.move_to_pose_from_transform(wp1)
		self.move_to_pose_from_transform(self.neutral_pos)
		self.move_to_pose_from_transform(self.halfway_pos)
		self.move_to_pose_from_transform(self.dump_pos)

	def move_to_pose_from_transform(self, trans, onTable=False, difference=.1):
		dest = PoseStamped()
		dest.header.frame_id = 'base'
		dest.pose.position.x = trans[0] 
		dest.pose.position.y = trans[1] 
		dest.pose.position.z = trans[2] 
		if onTable:
			dest.pose.position.x += difference
			dest.pose.position.z = self.minZ
		
		#Orientation as a quaternion
		dest.pose.orientation.x = 0
		dest.pose.orientation.y = -1
		dest.pose.orientation.z = 0
		dest.pose.orientation.w = 0

		#Set the goal state to the pose you just defined
		self.main_arm.set_pose_target(dest)
		#Set the start state for the left arm
		self.main_arm.set_start_state_to_current_state()
		
		#Create a path constraint for the arm
		orien_const = OrientationConstraint()
		orien_const.link_name = "left_gripper";
		orien_const.header.frame_id = "base";
		orien_const.orientation.y = -1.0;
		### Make sure to set these back to .1
		orien_const.absolute_x_axis_tolerance = .1;
		orien_const.absolute_y_axis_tolerance = .1;
		orien_const.absolute_z_axis_tolerance = .1;
		orien_const.weight = 1.0;
		consts = Constraints()
		consts.orientation_constraints = [orien_const]
		### Comment out this line if Baxter is having trouble with finding a path
		self.main_arm.set_path_constraints(consts)

		#Plan a path
		main_plan = self.main_arm.plan()

		#Execute the plan
		self.main_arm.execute(main_plan)

	def move_secondary_arm(self, trans, onTable=False):
		dest = PoseStamped()
		dest.header.frame_id = 'base'
		dest.pose.position.x = trans[0] 
		dest.pose.position.y = trans[1] 
		dest.pose.position.z = trans[2] 
		if onTable:
			dest.pose.position.x += .1
			dest.pose.position.z = self.minZ
		
		#Orientation as a quaternion
		dest.pose.orientation.x = 0
		dest.pose.orientation.y = -1
		dest.pose.orientation.z = 0
		dest.pose.orientation.w = 0

		#Set the goal state to the pose you just defined
		self.secondary_arm.set_pose_target(dest)
		#Set the start state for the left arm
		self.secondary_arm.set_start_state_to_current_state()
		
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
		self.secondary_arm.set_path_constraints(consts)

		#Plan a path
		secondary_plan = self.secondary_arm.plan()

		#Execute the plan
		self.secondary_arm.execute(secondary_plan)

	def get_zumy(self):
		x = input("wait for Zumy then enter some value")
		self.listener.waitForTransform('/base',zumy_tag, rospy.Time(0), rospy.Duration(5))
		(zumy_trans, _) = self.listener.lookupTransform('/base',zumy_tag,rospy.Time(0))
		rospy.sleep(3)
		grab_pt = np.add(zumy_trans, [-.07, 0, self.minZ+.1])
		self.move_to_pose_from_transform(grab_pt)
		self.move_to_pose_from_transform(zumy_trans, True, -.07)
		self.grab()
		self.move_to_blender_through_neutral()
		self.dump()

if __name__ == '__main__':
	bax = BlenderBaxter()
	bax.setup()
	bax.main(sys.argv)