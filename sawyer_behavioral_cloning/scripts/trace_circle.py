import tensorflow as tf
from PIL import Image as PILImage
from numpy import array
from numpy import ndarray
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from numpy import array
import random
import csv
import time
from random import randint
import intera_interface


import argparse

import rospy

import intera_interface
import rospkg

from std_msgs.msg import String
from std_msgs.msg import Bool

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from intera_core_msgs.msg import EndpointState

from intera_interface import CHECK_VERSION

import time
import datetime
import sys

from copy import deepcopy
import numpy as np
import csv

import random
from sawyer_demonstration.srv import *
from ros_falcon.msg import falconForces
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion

from gazebo_msgs.msg import ModelStates
import scipy.signal as signal

from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError
import cv2

from sensor_msgs.msg import JointState
from message_filters import TimeSynchronizer, Subscriber
import message_filters



rospy.init_node("circle_node")



class Environment(object):
	def __init__(self):
	
		self.limb = intera_interface.Limb('right')
		self._pose, self._point, self._q = self._set_initial_pose_of_eef()

		self.move_to_neutral()
		self.spawn_table()
		rospy.on_shutdown(self.delete_table)


	def _set_initial_pose_of_eef(self):
		# End effector orientation is fixed.
		q = Quaternion()
		q.x = 0.642161760993
		q.y = 0.766569045326
		q.z = 0.000271775440016
		q.w = 0.00031241683589

		point = Point()
		pose = Pose()

		pose.position = point
		pose.orientation = q

		return pose, point, q

	def spawn_table(self, table_pose=Pose(position=Point(x=0.55, y=0.0, z=0.0)),table_reference_frame="world"):
	        # Get Models' Path
	        model_path = rospkg.RosPack().get_path('sawyer_gazebo_env')+"/models/"
	        # Load Table URDF
	        table_xml = ''
	        with open (model_path + "table/table.urdf", "r") as table_file:
	            table_xml=table_file.read().replace('\n', '')
	        # Spawn Table URDF
	        rospy.wait_for_service('/gazebo/spawn_urdf_model')
	        try:
	            spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
	            resp_urdf = spawn_urdf("table", table_xml, "/",
	                                 table_pose, table_reference_frame)
	        except rospy.ServiceException, e:
	            rospy.logerr("Spawn SDF service call failed: {0}".format(e))

	def random_spawn_cube(self):
		x = random.uniform(0.5, 0.7) # 0.5, 0.7
		y = random.uniform(-0.2, 0.2)
		self.spawn_cube(x, y)
		return x, y

	def spawn_cube(self, _x, _y, block_reference_frame="world"):
		# block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725))
		block_pose=Pose(position=Point(x=_x, y=_y, z=0.825))
		# Get Models' Path
		model_path = rospkg.RosPack().get_path('sawyer_sim_examples')+"/models/"

		# Load Block URDF
		block_xml = ''
		with open (model_path + "block/model.urdf", "r") as block_file:
		    block_xml=block_file.read().replace('\n', '')

		# Spawn Block URDF
		rospy.wait_for_service('/gazebo/spawn_urdf_model')
		try:
		    spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
		    resp_urdf = spawn_urdf("block", block_xml, "/",
		                           block_pose, block_reference_frame)
		except rospy.ServiceException, e:
		    rospy.logerr("Spawn URDF service call failed: {0}".format(e))
		return _x, _y

	def delete_cube(self):
		try:
		    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
		    resp_delete = delete_model("block")
		except rospy.ServiceException, e:
		    print("Delete Model service call failed: {0}".format(e))

	def delete_table(self):
		try:
		    delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
		    resp_delete = delete_model("table")
		except rospy.ServiceException, e:
		    print("Delete Model service call failed: {0}".format(e))


	def move_to_neutral(self):
	        
	        self._point.x = 0.4
	        self._point.y = 0.0
	        self._point.z = 0.0
	        
	        self.limb.move_to_joint_positions(self.limb.ik_request(self._pose))


import math

# def circle(x, x_0, r):
# 	var = r**2 - (x - x_0)**2
# 	var = math.sqrt(var**2)
# 	y = math.sqrt(var)
# 	return y

def circle(x_0, y_0, r, theta):
	x = x_0 + r*math.cos(theta)
	y = y_0 + r*math.sin(theta)
	return x, y


import time
import math
rate = rospy.Rate(10)
# set the gazebo environment 
# to begin behavioral cloning
env = Environment()




x_0 = 0.6
y_0 = 0.0
r = 0.2

theta = 3.14

while not rospy.is_shutdown():

	
	x, y = circle(x_0, y_0, r, theta)
	
	env._point.x = x
	env._point.y = y

	if env.limb.ik_request(env._pose) != False:
		print x, y
		env.limb.move_to_joint_positions(env.limb.ik_request(env._pose))
	else:
		print "IK Request failed."
		env.move_to_neutral()

	# if x >= 0.8:
	# 	x = 0.4

	# else:
	# 	x += 0.04
	if theta <= 0.0:
		theta = 3.14

	else:
		theta += 0.1

	# time.sleep(1)
	
