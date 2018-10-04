#!/usr/bin/env python

# the script to generate expert demonstrations

import argparse

import rospy

import intera_interface
import rospkg

from std_msgs.msg import String

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
from intera_examples import JointRecorder

from intera_interface import CHECK_VERSION

import time
import datetime
import moveit_commander
import sys

from copy import deepcopy
import numpy as np

import random
from joystick import CustomController

rospy.init_node('demonstrator_node')

demonstrator_publisher_start = rospy.Publisher('/demonstrator/message/start', String, queue_size=10)
demonstrator_publisher_stop = rospy.Publisher('/demonstrator/message/stop', String, queue_size=10)

# recorder_subscriber_start = rospy.Subscriber('/recorder/message/start', String, ping_from_the_recorder_start_cb)
# recorder_subscriber_stop = rospy.Subscriber('/recorder/message/stop', String, ping_from_the_recorder_stop_cb)

flag = False

limb = intera_interface.Limb('right')
from geometry_msgs.msg import Pose
pose = Pose()
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
point = Point()

q = Quaternion()
q.x = 0.642161760993
q.y = 0.766569045326
q.z = 0.000271775440016
q.w = 0.00031241683589

pose.position = point
pose.orientation = q

current_x = 0.0
current_y = 0.0
current_z = 0.0

reset = False

def reset():
    delete_gazebo_models()
    x = (random.uniform(0.5, 0.7))
    y = (random.uniform(-0.4, 0.4))
    spawn_cube(x, y)
    reset = True


def spawn_cube(_x, _y, block_reference_frame="world"):
    print _x, _y
    # block_pose=Pose(position=Point(x=0.4225, y=0.1265, z=0.7725))
    block_pose=Pose(position=Point(x=_x, y=_y, z=0.7725))
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

def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("block")
    except rospy.ServiceException, e:
        print("Delete Model service call failed: {0}".format(e))


def move_to_neutral():

    point.x = 0.4
    point.y = 0.0
    point.z = 0.1
    
    limb.move_to_joint_positions(limb.ik_request(pose))

def ping_from_the_recorder_start_cb(data):
    print "recorder pings :", data


def ping_from_the_recorder_stop_cb(data):
    print "recorder pings :", data

def EndpointStateCb(data):
    current_x = data.pose.position.x
    current_y = data.pose.position.y
    current_z = data.pose.position.z

joystick = CustomController()
gripper = intera_interface.Gripper()

def main():
    rospy.Subscriber('/robot/limb/right/endpnt_state', EndpointState, EndpointStateCb)

    random.seed(1)
    move_to_neutral()
    gripper_open = False
    x = (random.uniform(0.5, 0.7))
    y = (random.uniform(-0.4, 0.4))
    spawn_cube(x, y)
    while not rospy.is_shutdown():
        # move_to_neutral()
        # point.x = x
        # point.y = y
        # point.z = 0.05
        # spawn_cube(x, y)

        if len(joystick._controls) > 0:
            # if leftBumper button is clicked : Open the gripper.
            if joystick._controls['leftBumper'] and (not gripper_open):
                gripper.open()
                gripper_open = True
            elif joystick._controls['leftBumper'] and gripper_open:
                gripper.close()
                gripper_open = False

            print joystick._controls['rightBumper']
            if joystick._controls['rightBumper'] and (not reset):
                reset()
                print "reset"

            if joystick._controls['leftStickHorz'] < 0.0:
                point.x += 0.01
                # print "right"
            elif joystick._controls['leftStickHorz'] > 0.0:
                point.x -= 0.01
                # print "left"

            if joystick._controls['leftStickVert'] < 0.0:
                point.y -= 0.01
                # print "down"
            elif joystick._controls['leftStickVert'] > 0.0:
                point.y += 0.01
                # print "up"

            if joystick._controls['rightStickVert'] > 0.0:
                point.z += 0.01
            elif joystick._controls['rightStickVert'] < 0.0:
                point.z -= 0.01

            if limb.ik_request(pose) != False:
                demonstrator_publisher_start.publish(str(datetime.datetime.time(datetime.datetime.now())))
                limb.move_to_joint_positions(limb.ik_request(pose))
                demonstrator_publisher_stop.publish('stop')     
            else:
                print "IK Request failed."
        # delete_gazebo_models()
    

if __name__ == '__main__':
    main()

    
