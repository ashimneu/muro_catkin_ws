#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
from pi_controller.msg import Speeds

from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point

from math import *
from numpy import matrix
from numpy import absolute

target_pose = Pose2D()
robot_pose = Pose2D()
transformScale=900;
transformX=301;
transformY=247;

def fake_target_pose_CB(pose):
	global target_pose
	target_pose = pose
	rospy.loginfo("\ntarget:\nx: %f\ny: %f\ntheta: %f\n", target_pose.x, target_pose.y, target_pose.theta)

def fake_robot_pose_CB(pose):
	global robot_pose
	robot_pose = pose
	rospy.loginfo("\nrobot:\nx: %f\ny: %f\ntheta: %f\n", pose.x, pose.y, pose.theta)
	if(robot_pose.x == 3 and robot_pose.y == 3 and robot_pose.theta == 3):
		motor_publisher(matrix([[0],[0],[0]]))
	else:	
		calc_pose_vector()

def target_pose_CB(pose):
	global target_pose
	target_pose.x = pose.pose.position.x
	target_pose.y = pose.pose.position.y
	target_pose.theta = 0
	rospy.loginfo("\ntarget:\nx: %f\ny: %f\ntheta: %f\n", target_pose.x, target_pose.y, target_pose.theta)

def robot_pose_CB(pose):
	global robot_pose
	robot_pose = pose
	robot_pose.x = (robot_pose.x-transformX)/transformScale
	robot_pose.y = (robot_pose.y-transformY)/transformScale
	rospy.loginfo("\nrobot:\nx: %f\ny: %f\ntheta: %f\n", pose.x, pose.y, pose.theta)
	rospy.loginfo("\ntarget:\nx: %f\ny: %f\ntheta: %f\n", target_pose.x, target_pose.y, target_pose.theta)

	calc_pose_vector()

def calc_pose_vector():
	pose_vector = Pose2D()
	pose_vector = subtract_pose(target_pose,robot_pose)
	#phi_init = robot_pose.theta;
	phi_init = 0;

	dist_to_target = sqrt(pose_vector.x**2+pose_vector.y**2)
	print pose_vector.x
	print pose_vector.y

	if abs(pose_vector.x) > .06 or abs(pose_vector.y) >.06:
		desired_xyphi = matrix([[-pose_vector.x],[pose_vector.y],[-robot_pose.theta/1.7]])
		wheel_output = calc_wheel_contribution(desired_xyphi,phi_init)
	else:
		desired_xyphi = matrix([[0],[0],[0]])
		#wheel_output = calc_wheel_contribution(desired_xyphi,phi_init)
		wheel_output = matrix([[0],[0],[0]])
	rospy.loginfo("\nDesired xyphi_dot:\nx_dot: %f\ny_dot: %f\nphi_dot: %f\n",\
															 desired_xyphi[0],\
															 desired_xyphi[1],\
															 desired_xyphi[2])
	
	motor_publisher(wheel_output)

def motor_publisher(wheel_output):
	pwm_msg = Speeds()
	pwm_msg.s1 = wheel_output[0].round(0)
	pwm_msg.s2 = wheel_output[1].round(0)
	pwm_msg.s3 = wheel_output[2].round(0)
	pwm_pub = rospy.Publisher('speeds', Speeds, queue_size=10)
	pwm_pub.publish(pwm_msg)
	rospy.loginfo("\nPublished to topic /speeds:\nMotor 1: %f\nMotor 2: %f\nMotor 3: %f\n",\
															 wheel_output[0].round(0),\
															 wheel_output[1].round(0),\
															 wheel_output[2].round(0))

def subtract_pose(t_pose,r_pose):
	diff_pose = Pose2D()
	diff_pose.x = t_pose.x - r_pose.x
	diff_pose.y = t_pose.y - r_pose.y
	diff_pose.theta = t_pose.theta - r_pose.theta
	return diff_pose

def calc_wheel_contribution(desired_xyphi,phi_init):
	d = pi/6; 		 #angle of front wheels from xaxis
	L = 74*10**-3;    #length from center to wheel
	#phi_init = 0;    #angle orientation of robot from 1 2 y pos
	R = 25*10**-3;
	max_pwm = 60;
	A_ss = 1/R*matrix([[-sin(d-phi_init), -cos(d-phi_init), L],\
					   [-sin(d+phi_init), cos(d+phi_init), L],\
					   [ cos(phi_init), sin(phi_init), L]])
	input_ss = desired_xyphi;
	output_ss = A_ss*input_ss
	abs_ss = absolute(output_ss)
	val_max = abs_ss.max()
	# if input_ss[0] == 0 and input_ss[0] == 0:
	# 	max_pwm = 10;

	#if val_max == 0:
	#	norm_ss = matrix([[0],[0],[0]])
	#else:
	scale_factor = max_pwm/val_max
	norm_ss = scale_factor*output_ss
	# print A_ss
	# print input_ss
	# print output_ss
	return(norm_ss)

def main():

	rospy.init_node('motor_handler', anonymous=True)
   
	rospy.Subscriber("aruco/robot_pose", Pose2D, fake_robot_pose_CB)
	# rospy.Subscriber("aruco/target_pose", Pose2D, fake_target_pose_CB)
	rospy.Subscriber("goal_pose", PoseStamped, target_pose_CB)
	# rospy.Subscriber("chip/location", Pose2D, robot_pose_CB)

	while not rospy.is_shutdown():
		rospy.spin()
		


if __name__ == '__main__':
	main()
	# test_input = matrix([[0],[1],[pi/3]])
	# calc_wheel_contribution(test_input)