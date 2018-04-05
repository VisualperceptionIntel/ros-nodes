#!/usr/bin/env python
"""************************************************************************************************
***                                                                                             ***
***                                      ###############                                        ***
***                              ##     #################        ##                             ***
***                         #######    ####################      #######                         ***
***                     ##########     #####################      ##########                    ***
***                  ############     #######      ##########     ############                  ***
***           ####   ############     ######          ########     ############   ####          ***
***      ########   #############     ######           #######     #############   ########     ***
***   ##########    #############     ######            ######     #############   ##########   ***
***   ##########    #############     ######            ######     #############   ##########   ***
***      ########   #############     ######           #######     #############   ########     ***
***           ####   ############     ######          ########     ############   ####          ***
***                   ############    #######       #########     ############                  ***
***                     ##########     #####################      ##########                    ***
***                         #######    ###################       #######                         ***
***                              ##     ################         ##                             ***
***                                      #############                                          ***
***                   ____  ___  ____   ____   ___ __     __ _____  ____   ____                 ***
***                  |  _ \|_ _||  _ \ |  _ \ |_ _|\ \   / /| ____||  _ \ / ___|                ***
***                  | | | || | | | | || |_) | | |  \ \ / / |  _|  | |_) |\___ \                ***
***                  | |_| || | | |_| ||  _ <  | |   \ V /  | |___ |  _ <  ___) |               ***
***                  |____/|___||____/ |_| \_\|___|   \_/   |_____||_| \_\|____/                ***
***                                                                                             ***
***             ACCELERATING THE WORLDS TRANSITION TO AUTONOMOUS VEHICLES AND ROBOTICS          ***
***                                                                                             ***
***************************************************************************************************
*
*  File: obstacle_detection.py
*  Desc: Subscribes to topics specified on YAML file, creates a listener object for each topic
*        and calculates the closest obstacle within the configured window
*        
*  Maintainers: Mahindan
*               
*
*  Subscribes:
*      - Configured on YAML file
*
*  Publishes:
*      - Configured on YAML, if not set then defaulta to: 
*         topicOutFront: "/obstacle/front" 
*         topicOutBack: "/obstacle/back" 
*         topicOutLeft: "/obstacle/left" 
*         topicOutRight: "/obstacle/right"
*
*                                                                                     *
*  Copyright (c) 2017, DiDrivers Limited.                                                         *
*  All Rights Reserved                                                                            *
*                                                                                                 *
*  Redistribution and use in source and binary forms, with or without                             *
*  modification, are permitted provided that the following conditions are met:                    *
*      * Redistributions of source code must retain the above copyright                           *
*        notice, this list of conditions and the following disclaimer.                            *
*      * Redistributions in binary form must reproduce the above copyright                        *
*        notice, this list of conditions and the following disclaimer in the                      *
*        documentation and/or other materials provided with the distribution.                     *
*      * Neither the name of DiDrivers Limited. nor the                                           *
*        names of its contributors may be used to endorse or promote products                     *
*        derived from this software without specific prior written permission.                    *
*                                                                                                 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          *
* DISCLAIMED. IN NO EVENT SHALL Didrivers Limited. BE LIABLE FOR ANY                              *
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                      *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     *
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    *
*                                                                                                 *
* Please send comments, questions, or patches to support@didrivers.com                            *
************************************************************************************************"""
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from wizo_msgs.msg import Ultrasonic
import datetime
import yaml
from termcolor import colored
import math
import rospkg


now = datetime.datetime.now()
'''pub_obs_front = rospy.Publisher('left_arm_joint_cmd', kinova_msgs.msg.JointAngles, queue_size=1)
pub_obs_back  = rospy.Publisher('right_arm_joint_cmd', kinova_msgs.msg.JointAngles, queue_size=1)
pub_obs_right = rospy.Publisher('/arm_action_status', String, queue_size=1)
pub_obs_left  = rospy.Publisher('/arm_action_status', String, queue_size=1)'''

# Initiate Class for Listeners. i.e for all topics set on YAML and respective type
class ListenerClass:
	
	#Class variables common for all instance, therefore will always be the closest obstacle for all subscribers
	obs_front = [99,99,99]
	obs_back  = [-99,-99,-99]
	obs_left  = [99,99,99]
	obs_right = [-99,-99,-99]
	
	def __init__(self,topic):
		#Set the subscribers based on topic type on YAML
		if 'Int16' in topic['topic_type']: 
			if 'sensor_config' in topic:
				self.sensor_position = topic['sensor_config']	
				self.string_sub = rospy.Subscriber(topic['topic_name'], Int16, self.intType_cb)
		elif 'Float32' in topic['topic_type']: 
			self.string_sub = rospy.Subscriber(topic['topic_name'], Float32, self.floatType_cb)
		elif 'PointCloud2' in topic['topic_type']: 
			self.string_sub = rospy.Subscriber(topic['topic_name'], PointCloud2, self.pointCloudType_cb)
		elif 'Ultrasonic' in topic['topic_type']: 
			if 'ultra_config' in topic:
				self.ultrasonics_position = topic['ultra_config']		
			self.string_sub = rospy.Subscriber(topic['topic_name'], Ultrasonic, self.ultrasonicType_cb)
		
#Set range conditions			
		#                    Z
		#                    ^ (0 Deg)
		#               -----|---X------
		#              -     |  ^     -
		#  (90 Deg)   -      | /     - (-90 Deg)
		#            -       |/     -
		#           -  Y <---o     - 
		#          ----------------
		#               (180 Deg)
		#
		#
		############### max ################
		# #      |               |       # #
		# #     w/2    (F)     -w/2      # #
		# # __w__|               |___w__ # #
		# #   2                      2   # #
		# #        **** min ****         # #
		# #        *           *         # #
		# #        *    ###    *         # # 
		#max      min   ###   min       max# 
		# #  (L)   *    ###    *   (R)   # #   
		# #        *           *         # #
		# #        **** min ****         # #
		# # _-w__                 __-w__ # #
		# #   2  |     (B)       |   2   # #
		# #     w/2            -w/2      # #
		# #      |               |       # #
		############### max ################
		
		#---------------------------------------------------------------------------------------------------#
		if 'min' in topic['collision_view_front']:
			self.front_min = topic['collision_view_front']['min']
		if 'max' in topic['collision_view_front']:
			self.front_max = topic['collision_view_front']['max']
		if 'width' in topic['collision_view_front']:
			self.front_width = topic['collision_view_front']['width']
		if 'height_min' in topic['collision_view_front']:
			self.front_height_min = topic['collision_view_front']['height_min']
		if 'height_max' in topic['collision_view_front']:
			self.front_height_max = topic['collision_view_front']['height_max']
		#---------------------------------------------------------------------------------------------------#
		if 'min' in topic['collision_view_back']:
			self.back_min = -1 * topic['collision_view_back']['min']
		if 'max' in topic['collision_view_back']:
			self.back_max = -1 * topic['collision_view_back']['max']
		if 'width' in topic['collision_view_back']:
			self.back_width = topic['collision_view_back']['width']
		if 'height_min' in topic['collision_view_back']:
			self.back_height_min = topic['collision_view_back']['height_min']
		if 'height_max' in topic['collision_view_back']:
			self.back_height_max = topic['collision_view_back']['height_max']
		#---------------------------------------------------------------------------------------------------#
		if 'min' in topic['collision_view_left']:
			self.left_min = topic['collision_view_left']['min']
		if 'max' in topic['collision_view_left']:
			self.left_max = topic['collision_view_left']['max']
		if 'width' in topic['collision_view_left']:
			self.left_width = topic['collision_view_left']['width']
		if 'height_min' in topic['collision_view_left']:
			self.left_height_min = topic['collision_view_left']['height_min']
		if 'height_max' in topic['collision_view_left']:
			self.left_height_max = topic['collision_view_left']['height_max']
		#---------------------------------------------------------------------------------------------------#
		if 'min' in topic['collision_view_right']:
			self.right_min = -1 * topic['collision_view_right']['min']
		if 'max' in topic['collision_view_right']:
			self.right_max = -1 * topic['collision_view_right']['max']
		if 'width' in topic['collision_view_right']:
			self.right_width = topic['collision_view_right']['width']
		if 'height_min' in topic['collision_view_right']:
			self.right_height_min = topic['collision_view_right']['height_min']
		if 'height_max' in topic['collision_view_right']:
			self.right_height_max = topic['collision_view_right']['height_max']
		#---------------------------------------------------------------------------------------------------#
		
			
	def intType_cb(self,msg):
		int_dist = msg.data
		if int_dist >= self.sensor_position['min'] and int_dist <= self.sensor_position['max']:
					# Calculate corrdinates based on position and angle
					theta =  self.sensor_position['yaw']
					p_x = float("%.2f" % (self.sensor_position['x'] + int_dist * math.cos(math.radians(theta))))
					p_y = float("%.2f" % (self.sensor_position['y'] + int_dist * math.sin(math.radians(theta))))
					p_z = float(self.sensor_position['z'])
					p = [p_x,p_y,p_z]
					
					#Check if this detection is the closest 
					self.checkClosPoint(p)
					
	def floatType_cb(self,msg):
		float_dist = msg.data
		if float_dist >= self.sensor_position['min'] and float_dist <= self.sensor_position['max']:
					# Calculate corrdinates based on position and angle
					theta =  self.sensor_position['yaw']
					p_x = float("%.2f" % (self.sensor_position['x'] + float_dist * math.cos(math.radians(theta))))
					p_y = float("%.2f" % (self.sensor_position['y'] + float_dist * math.sin(math.radians(theta))))
					p_z = float(self.sensor_position['z'])
					p = [p_x,p_y,p_z]
					
					#Check if this detection is the closest 
					self.checkClosPoint(p)
	def pointCloudType_cb(self,scan):
		for p in pc2.read_points(scan, field_names = ("x", "y", "z"), skip_nans=True):
			us_x = float("%.2f" % p[0])
			us_y = float("%.2f" % p[1])
			us_z = float("%.2f" % p[2])
			p = [us_x,us_y,us_z]
			self.checkClosPoint(p)

	def ultrasonicType_cb(self,msg):	
		#get ultrasonic list message and split into individual detection
		us_str_msg = str(msg).split('\n')
		#For each detection, check if configured on YAML. 
		for index in range(0,len(us_str_msg)):
			temp_str=us_str_msg[index].split(": ")
			us_index = str(temp_str[0])
			us_val = float("%.2f" % float(temp_str[1]))
			#If particular ultrasonic detection is configured, 
			if us_index in self.ultrasonics_position:
				# check if detection is within optimum range as specified
				if us_val >= self.ultrasonics_position[us_index]['min'] and us_val <= self.ultrasonics_position[us_index]['max']:
					# Calculate corrdinates based on position and angle
					theta =  self.ultrasonics_position[us_index]['yaw']
					us_x = float("%.2f" % (self.ultrasonics_position[us_index]['x'] + us_val * math.cos(math.radians(theta))))
					us_y = float("%.2f" % (self.ultrasonics_position[us_index]['y'] + us_val * math.sin(math.radians(theta))))
					us_z = float(self.ultrasonics_position[us_index]['z'])
					p = [us_x,us_y,us_z]
					
					#Check if this detection is the closest 
					self.checkClosPoint(p)
				

					

	def checkClosPoint(self,p):
			# Define XYZ to be used in index
			X = 0
			Y = 1
			Z = 2
			
			#Check closest obstacle based on range conditions set on YAML
			if p[X] >=self.front_min and p[X] <= self.front_max and p[Y] >= -1 * self.front_width/2 and p[Y] <= self.front_width/2 and  p[Z] >=self.front_height_min and p[Z] <= self.front_height_max :
				if p[X] < ListenerClass.obs_front[X]: 
					ListenerClass.obs_front = p
					#print '------------------------------------------F'
			if p[X] <=self.back_min and p[X] >= self.back_max and p[Y] >= -1 * self.back_width/2 and p[Y] <= self.back_width/2 and  p[Z] >=self.back_height_min and p[Z] <= self.back_height_max :
				if p[X] > ListenerClass.obs_back[X]: 
					ListenerClass.obs_back = p
					#print '------------------------------------------B'
			if p[Y] >=self.left_min and p[Y] <= self.left_max and p[X] >= -1 * self.left_width/2 and p[X] <= self.left_width/2 and  p[Z] >=self.left_height_min and p[Z] <= self.left_height_max :
				if p[Y] < ListenerClass.obs_left[Y]: 
					ListenerClass.obs_left = p
					#print '------------------------------------------L'
			if p[Y] <=self.right_min and p[Y] >= self.right_max and p[X] >= -1 * self.right_width/2 and p[X] <= self.right_width/2 and  p[Z] >=self.right_height_min and p[Z] <= self.right_height_max :
				if p[Y] > ListenerClass.obs_right[Y]: 
					ListenerClass.obs_right = p
					#print '------------------------------------------R'
##############################################################################################																								 					


		
##############################################################################################												
def obstacleDetection():
	#Initialise the node
	rospy.init_node('obstacle_detection')
	
	# Get the configurations from obstacle-param.yaml File unless filename specified on roslaunch
	print 'Getting configurations now'
	# Get an instance of RosPack with the default search paths
	rospack = rospkg.RosPack()
	obs_configurations =''
	
	try:
		obs_configurations = rospy.get_param('obstacle_detection')
	except:
		pass

	while not obs_configurations:
		obs_configurations_file_name = str(rospack.get_path('configurations')) + '/config/obstacle-param.yaml'			
		obstacle_param_yaml_file = open(obs_configurations_file_name,"r")
		#Load the YAML config
		obs_configurations = yaml.load(obstacle_param_yaml_file)['obstacle_detection']
		if obs_configurations is '':
			print 'configurations file not found, trying every second...'
			rospy.sleep(1)
	# Define XYZ to be used in index
	X = 0
	Y = 1
	Z = 2
	
	# Configure the subscribers and Publisher as specified on YAML
	for params in obs_configurations:
		if 'topics_in' in 	params:	
			for topics_in_config in params['topics_in']:
				#Create Listener objects for each topic and pass list (topics_in_config) of parameters for that topic
				ListenerClass(topics_in_config)
		if 'topics_out' in 	params:	
			#Configure the Publisher with name configured on YAML
			topics_out_list = params['topics_out']
			#If name not specified then default
			#---------------------------------------------------------------------------------------------------#
			if 'topicOutFront' in topics_out_list:
				pub_obs_front = rospy.Publisher(str(topics_out_list['topicOutFront']), Point, queue_size=1)
			else:
				pub_obs_front = rospy.Publisher('/obstacle/front', Point, queue_size=1)
			#----------------------------------------------------------------------------#
			if 'topicOutBack' in topics_out_list:
				pub_obs_back  = rospy.Publisher(str(topics_out_list['topicOutBack']), Point, queue_size=1)
			else:
				pub_obs_back = rospy.Publisher('/obstacle/back', Point, queue_size=1)
			#----------------------------------------------------------------------------#
			if 'topicOutLeft' in topics_out_list:
				pub_obs_right = rospy.Publisher(str(topics_out_list['topicOutLeft']), Point, queue_size=1)
			else:
				pub_obs_right = rospy.Publisher('/obstacle/left', Point, queue_size=1)
			#----------------------------------------------------------------------------#
			if 'topicOutRight' in topics_out_list:
				pub_obs_left  = rospy.Publisher(str(topics_out_list['topicOutRight']), Point, queue_size=1)
			else:
				pub_obs_left = rospy.Publisher('/obstacle/right', Point, queue_size=1)
			#---------------------------------------------------------------------------------------------------#	
		
		#Load other configs
		#TODO: set deafault min / max ranges independant of topics if not given
		if 'config' in 	params:		
			rate = params['config']['rate']
			r = rospy.Rate(rate)
			

	while not rospy.is_shutdown():
		print colored('FRONT: ','white') + colored(" x : %f  y: %f  z: %f" %(ListenerClass.obs_front[X],ListenerClass.obs_front[Y],ListenerClass.obs_front[Z]),'green') 
		print colored('Left : ','white') + colored(" x : %f  y: %f  z: %f" %(ListenerClass.obs_left[X],ListenerClass.obs_left[Y],ListenerClass.obs_left[Z]),'yellow')
		print colored('Right: ','white') + colored(" x : %f  y: %f  z: %f" %(ListenerClass.obs_right[X],ListenerClass.obs_right[Y],ListenerClass.obs_right[Z]),'yellow')
		print colored('BACK : ','white') + colored(" x : %f  y: %f  z: %f" %(ListenerClass.obs_back[X],ListenerClass.obs_back[Y],ListenerClass.obs_back[Z]),'red')
		print('##########################################################################\n')	
		
		#Get the obstacles from class variable(Common var for all instance) and publish 
		#---------------------------------------------------------------------------------------------------#	
		msg_point = Point()
		msg_point.x = ListenerClass.obs_front[X]
		msg_point.y = ListenerClass.obs_front[Y]
		msg_point.z = ListenerClass.obs_front[Z]
		pub_obs_front.publish(msg_point)
		#----------------------------------------------------#	
		msg_point.x = ListenerClass.obs_back[X]
		msg_point.y = ListenerClass.obs_back[Y]
		msg_point.z = ListenerClass.obs_back[Z]
		pub_obs_back.publish(msg_point)
		#----------------------------------------------------#	
		msg_point.x = ListenerClass.obs_left[X]
		msg_point.y = ListenerClass.obs_left[Y]
		msg_point.z = ListenerClass.obs_left[Z]
		pub_obs_left.publish(msg_point)
		#----------------------------------------------------#			
		msg_point.x = ListenerClass.obs_right[X]
		msg_point.y = ListenerClass.obs_right[Y]
		msg_point.z = ListenerClass.obs_right[Z]
		pub_obs_right.publish(msg_point)
		#---------------------------------------------------------------------------------------------------#	
		
		#Reset obstacle to retrieve in next loop
		ListenerClass.obs_front = [99,99,99]
		ListenerClass.obs_back  = [-99,-99,-99]
		ListenerClass.obs_left  = [99,99,99]
		ListenerClass.obs_right = [-99,-99,-99]
		
		#Rate set on YAML
		r.sleep()										

if __name__ == '__main__':
    try:
        obstacleDetection()

    except rospy.ROSInterruptException:
	pass

