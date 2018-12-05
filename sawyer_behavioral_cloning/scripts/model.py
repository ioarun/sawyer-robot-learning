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

random.seed(random.randint(0, 1000000))

rospy.init_node("bc_node")


# actual image dimension is 800x800
img_size = 200
# convert it into an input vector of 800*800*800 = 1920000 dimension
img_size_flat = img_size * img_size * img_size

# number of channels
num_channels = 3

# conv 1
filter_size1 = 7      # 7x7 filter dimension
num_filters1 = 32     # 64 filters in layer 1
stride1 = 2

# conv 2
filter_size2 = 5
num_filters2 = 32
stride2 = 1

# conv 3
filter_size3 = 5
num_filters3 = 32
stride3 = 1

number_features = 64
number_robot_config = 3
fc_size = 40
number_out = 2 # eef pose x, y, z

beta = 0.01

counter = 0
# tensorflow computation graph begins
training = tf.placeholder(tf.bool)

# helper functions for creating new weights and biases
def new_weights(shape):
	return tf.Variable(tf.truncated_normal(shape, stddev=0.05))

def new_biases(length):
	return tf.Variable(tf.constant(0.05, shape=[length]))

def new_conv_layer(input, num_input_channels, filter_size, num_filters, stride, use_pooling=True):

	# shape of the each filter-weights
	shape = [filter_size, filter_size, num_input_channels, num_filters]

	# create new weights
	weights = new_weights(shape=shape)

	# create new biases
	biases = new_biases(length=num_filters)

	# convolution operation
	# strides = 1, padding = 1 (to maintain spatial size same as previous layer)
	layer = tf.nn.conv2d(input=input, filter=weights, strides=[1,stride,stride,1], padding='SAME')

	# add biases to the results of convolution to each filter
	layer += biases

	if use_pooling:
		# 2x2 max-pooling
		layer = tf.nn.max_pool(value=layer, ksize=[1,2,2,1], strides=[1,2,2,1], padding='SAME')


	# batch normalization
	# layer = tf.layers.batch_normalization(layer, training=training)

	# ReLU operation, max(x, 0)
	layer = tf.nn.relu(layer)
	# layer = tf.layers.batch_normalization(layer, training=training)

	# layer = tf.layers.dropout(layer, rate=0.5, training=training)

	return layer, weights



# flatten layer for fully connected neural net
def flatten_layer(layer):
	# get shape of the input layer
	layer_shape = layer.get_shape()

	# layer shape is of the form [num_images, img_height, img_width, num_channels]
	# num_features = img_height*img_width*num_channels
	num_features = layer_shape[1:4].num_elements()

	layer_flat = tf.reshape(layer, [-1, num_features])
	# layer_flat = tf.layers.dropout(layer_flat, rate=0.125, training=training)


	return layer_flat, num_features

# create fully connected layer
def new_fc_layer(input, num_inputs, num_outputs, use_relu=True):
	# weights and biases for fc layer
	weights = new_weights(shape=[num_inputs, num_outputs])
	biases = new_biases(length=num_outputs)

	# linear operation
	layer = tf.matmul(input, weights) + biases

	if use_relu:
		# batch normalization
		# layer = tf.layers.batch_normalization(layer, training=training)
		layer = tf.nn.relu(layer)
		# layer = tf.layers.dropout(layer, rate=0.25, training=training)
		
	
	return layer, weights

train_batch_size = 1

x = tf.placeholder(tf.float32, shape=[train_batch_size, img_size, img_size, num_channels], name='x')


robot_config = tf.placeholder(tf.float32, shape=[train_batch_size, number_robot_config], name='robot_config')
# conv layers require image to be in shape [num_images, img_height, img_weight, num_channels]
x_image = tf.reshape(x, [-1, img_size, img_size, num_channels])


with tf.name_scope("cnn"):

	# conv layer 1
	layer_conv1, weights_conv1 = new_conv_layer(input=x_image,
											num_input_channels=num_channels,
											filter_size=filter_size1,
											num_filters=num_filters1,
											stride=stride1,
											use_pooling=False)
	# conv layer 2
	layer_conv2, weights_conv2 = new_conv_layer(input=layer_conv1,
											num_input_channels=num_filters1,
											filter_size=filter_size2,
											num_filters=num_filters2,
											stride=stride2,
											use_pooling=False)

	# conv layer 0
	layer_conv0, weights_conv0 = new_conv_layer(input=layer_conv2,
											num_input_channels=num_filters2,
											filter_size=filter_size3,
											num_filters=num_filters3,
											stride=stride3,
											use_pooling=False)

	# conv layer 3
	layer_conv3, weights_conv3 = new_conv_layer(input=layer_conv0,
											num_input_channels=num_filters3,
											filter_size=filter_size3,
											num_filters=num_filters3,
											stride=stride3,
											use_pooling=False)



	# feature_keypoints, number_features = flatten_layer(layer_conv3)


	# spatial softmax layer
	feature_keypoints = tf.contrib.layers.spatial_softmax(layer_conv3,
											temperature=None,
											name=None,
											variables_collections=None,
											trainable=True,
											data_format='NHWC')



	features_with_robot_config = tf.concat([feature_keypoints, robot_config], -1)


	# fully connected layer 1
	layer_fc1, fc1_weights = new_fc_layer(input=feature_keypoints,
						num_inputs=number_features,
						num_outputs=fc_size,
						use_relu=True)

	# fully connected layer 2
	layer_fc2, fc2_weights = new_fc_layer(input=layer_fc1,
						num_inputs=fc_size,
						num_outputs=fc_size,
						use_relu=True)
    
    # fully connected layer 2
	layer_fc3, fc3_weights = new_fc_layer(input=layer_fc2,
						num_inputs=fc_size,
						num_outputs=number_out,
						use_relu=False)



extra_update_ops = tf.get_collection(tf.GraphKeys.UPDATE_OPS)


class Environment(object):
	def __init__(self):
	
		self.limb = intera_interface.Limb('right')
		self._pose, self._point, self._q = self._set_initial_pose_of_eef()

		self.move_to_neutral()
		self.spawn_table()
		rospy.on_shutdown(self.delete_table)

		# Instantiate CvBridge
		self.bridge = CvBridge()
		self.cv2_img = None
		self.cv2_img_depth = None
		self.robot_config = []
		self.joints_right = self.limb.joint_names()
	
		self.camera_sub_rgb = rospy.Subscriber('/top_camera/camera/image_raw', Image, self._handle_image_cb)

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

	def _get_current_image(self):
		
		return (self.cv2_img)
		# return (self.cv2_img)

	def _get_robot_config(self):
		
		eef_pose = self.limb.endpoint_pose()
		# eef_vel = self.limb.endpoint_velocity()
		robot_conf = [(eef_pose['position']).x, (eef_pose['position']).y, (eef_pose['position']).z]
		angles_right = [self.limb.joint_angle(j) for j in self.joints_right]
	
		# for x in angles_right:
		# 	robot_conf.append(x)
			
		# self.robot_config = [robot_config]
		return [robot_conf]

		# return [[(eef_pose['position']).x, (eef_pose['position']).y, (eef_pose['position']).z,\
		# (eef_vel['linear']).x, (eef_vel['linear']).y, (eef_vel['linear']).z]]


	def get_input_data(self):
		image_input = self._get_current_image()
		robot_conf = self._get_robot_config()
		return image_input, [array(image_input)], robot_conf


	def _handle_image_cb(self, msg):
		
		try:
			# Convert your ROS Image message to OpenCV2
			self.cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

		except CvBridgeError, e:
		    print(e)

	def _handle_image_depth_cb(self, msg):
		
		try:
		    # Convert your ROS Image message to OpenCV2
			self.cv2_img_depth = self.bridge.imgmsg_to_cv2(msg)
			# cv2.imwrite("/home/arun/depth.jpg", self.cv2_img_depth)
		except CvBridgeError, e:
		    print(e)

	def _image_cb(self,msg_rgb, msg_depth): # TODO still too noisy!
		try:
			# The depth image is a single-channel float32 image
			# the values is the distance in mm in z axis
			cv_image = self.bridge.imgmsg_to_cv2(msg_depth, "32FC1")
			
			
			# Convert the depth image to a Numpy array since most cv2 functions
			# require Numpy arrays.
			cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
			# Normalize the depth image to fall between 0 (black) and 1 (white)
			# http://docs.ros.org/electric/api/rosbag_video/html/bag__to__video_8cpp_source.html lines 95-125
			cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
			# Resize to the desired size
			cv_image_resized = cv2.resize(cv_image_norm, (200,200), interpolation = cv2.INTER_CUBIC)
			self.cv2_img_depth = cv_image_resized
			# cv2.imwrite("/home/arun/depth2.png", self.cv2_img_depth)
			cv2.imshow("Image from my node", self.cv2_img_depth)
			cv2.waitKey(1)
		except CvBridgeError as e:
  			print(e)



import math

def plotNNFilter(units, index):
	filters = units.shape[3]
	plt.figure(index, figsize=(20,20))
	n_columns = 6
	n_rows = math.ceil(filters / n_columns) + 1


	for i in range(filters):
		plt.subplot(n_rows, n_columns, i+1)
		# plt.title('Filter ' + str(i))

		list_ = units[0,:,:,i]
		sum_of_activations = []
		softmax_layer = []
		temp =[]
		all_activations = []

		for i in range(len(list_)):
			for j in range(len(list_[0])):
				all_activations.append(list_[i][j])

		exp_all_activations = []

		for a in all_activations:
			exp_all_activations.append(math.exp(a))

		for i in range(len(list_)):
			temp =[]
			for j in range(len(list_[0])):
				temp.append(math.exp(list_[i][j])/sum(exp_all_activations))

			softmax_layer.append(temp)

		plt.imshow(softmax_layer, interpolation="nearest", cmap="gray")

	plt.show()


import time
import math
rate = rospy.Rate(10)
# set the gazebo environment 
# to begin behavioral cloning
env = Environment()




sess = tf.Session()

saver = tf.train.Saver()
# sess.run(tf.global_variables_initializer())
saver.restore(sess, "models/model.ckpt")

import math

while not rospy.is_shutdown():

	env.move_to_neutral()
	env.random_spawn_cube()

	time.sleep(2)

	input_image, x_batch, robot_config_ = env.get_input_data()
	
	# print input_image
	feed_dict_test = {x: x_batch, robot_config: robot_config_, training: False}

	action, f, extra, c3 = sess.run([layer_fc3, feature_keypoints, extra_update_ops, layer_conv3], feed_dict=feed_dict_test)
	print ("Network target prediction x: ", action[0][0]," y: ",action[0][1])
	print "---------------------------------------\n"

	# # plotNNFilter(c2, 1)
	# plotNNFilter(c3, 2)
	
	env._point.x = action[0][0]
	env._point.y = action[0][1]
	env._point.z = -0.100
	

	if env.limb.ik_request(env._pose) != False:
		
		env.limb.move_to_joint_positions(env.limb.ik_request(env._pose))
	else:
		print "IK Request failed."
		env.move_to_neutral()
	

	# rate.sleep()
	env.delete_cube()

