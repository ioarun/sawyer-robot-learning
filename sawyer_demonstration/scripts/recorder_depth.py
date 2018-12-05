import rospy

import intera_interface

from intera_interface import CHECK_VERSION

from std_msgs.msg import String
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

from sawyer_demonstration.srv import *
from message_filters import TimeSynchronizer, Subscriber
import message_filters


class DataRecorder(object):
    def __init__(self, rate, side="right"):
        """
        Records joint data to a file at a specified rate.
        """
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = True
        self.counter = 0
        self.is_recording = False
        self.filename = str(self.counter)+'.csv'
        self.cv2_img_rgb = None
        self.cv2_img_depth = None
        self.img_counter = 552
        self.cube_pose_x = 0.0
        self.last_cube_pose_x = 0.0
        self.cube_pose_y = 0.0

        self.last_eef_pose_x = 0.0

        self._limb_right = intera_interface.Limb(side)
        joints_right = self._limb_right.joint_names()

        self.rgb_image_path = 'data_rgb_3/'
        if not os.path.exists(self.rgb_image_path):
            os.makedirs(self.rgb_image_path)
        self.depth_image_path = 'data_depth/'
        if not os.path.exists(self.depth_image_path):
            os.makedirs(self.depth_image_path)


        with open(self.rgb_image_path+self.filename, 'w') as f:
            f.write('counter,')
            f.write('time,')
            f.write('cube_pose_x,')
            f.write('cube_pose_y,')
            f.write(','.join([j for j in joints_right]) + ',')
            f.write('eef_pose_x,')
            f.write('eef_pose_y,')
            f.write('eef_pose_z,')
            f.write('eef_vel_x,')
            f.write('eef_vel_y,')
            f.write('eef_vel_z,')
            
            f.write('\n')

    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def start(self):
        self._done =  False

    def stop(self):
        self._done = True  



    def record(self):
        ## for recording images and corresponding
        ## position labels. Supposed to be a very
        ## simple task

        joints_right = self._limb_right.joint_names()

        with open(self.rgb_image_path+self.filename, 'a') as f:
           
            while not dr._done:


                eef_pose = self._limb_right.endpoint_pose()['position']
                joints_right = self._limb_right.joint_names()

                if self.cube_pose_x != self.last_cube_pose_x:
                # if eef_pose.x != self.last_eef_pose_x: #to avoid redundant data when starting new demo

					
					eef_pose_x = eef_pose.x
					eef_pose_y = eef_pose.y
					eef_pose_z = eef_pose.z

					eef_vel = self._limb_right.endpoint_velocity()['linear']
					eef_vel_x = eef_vel.x
					eef_vel_y = eef_vel.y
					eef_vel_z = eef_vel.z

					f.write(str(self.img_counter)+',')
					f.write("%f," % (self._time_stamp(),))

					f.write(str(self.cube_pose_x)+',')
					f.write(str(self.cube_pose_y)+',')

					angles_right = [self._limb_right.joint_angle(j) for j in joints_right]
					f.write(','.join([str(x) for x in angles_right]) + ',')

					f.write(str(eef_pose_x)+',')
					f.write(str(eef_pose_y)+',')
					f.write(str(eef_pose_z)+',')

					f.write(str(eef_vel_x)+',')
					f.write(str(eef_vel_y)+',')
					f.write(str(eef_vel_z)+',')

					f.write('\n')

					# save current snapshot
					save_rgb_image()
					# save_depth_image()

					self.img_counter += 1

					self.last_eef_pose_x = eef_pose_x
					self.last_cube_pose_x = self.cube_pose_x


                self._rate.sleep()

                

def save_depth_image(): 
	try:

		cv_image_array = np.array(dr.cv2_img_depth, dtype = np.dtype('f8'))
		# Normalize the depth image to fall between 0 (black) and 1 (white)
		# http://docs.ros.org/electric/api/rosbag_video/html/bag__to__video_8cpp_source.html lines 95-125
		cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
		# Resize to the desired size
		cv_image_resized = cv2.resize(cv_image_norm, (200,200), interpolation = cv2.INTER_CUBIC)
		dr.cv2_img_depth = cv_image_resized
		cv2.imwrite(dr.depth_image_path+str(dr.img_counter)+'_depth.png', dr.cv2_img_depth)

	except:
		print( "error saving depth image" )

def save_rgb_image():
	try:
		cv2.imwrite(dr.rgb_image_path+str(dr.img_counter)+'_rgb.jpeg', dr.cv2_img_rgb)
	except:
		print( "error saving rgb image" )




def handle_start_recording(req):
    # print "returning start"
    dr.start()
    return StartRecordingResponse()

def handle_stop_recording(req):
    # print "returning stop"
    dr.stop()
    # dr.counter += 1
    # dr.filename = str(dr.counter)+'.csv'
    return StopRecordingResponse()


def handle_model_states_cb(msg):
    if "block" in msg.name:
        dr.cube_pose_x = round(msg.pose[4].position.x, 3)
        dr.cube_pose_y = round(msg.pose[4].position.y, 3)

def handle_image_cb(msg_rgb):
	# print "rgb :", msg_rgb, "depth :", msg_depth
	try:
		dr.cv2_img_rgb = bridge.imgmsg_to_cv2(msg_rgb, "bgr8")
		# dr.cv2_img_depth = bridge.imgmsg_to_cv2(msg_depth, "32FC1")
	except CvBridgeError, e:
		print(e)




rospy.init_node('joint_recorder_node')

# Instantiate CvBridge
bridge = CvBridge()

dr = DataRecorder(10)
s_start = rospy.Service('start_recording', StartRecording, handle_start_recording)
s_stop = rospy.Service('stop_recording', StopRecording, handle_stop_recording)
camera_sub = rospy.Subscriber('/top_camera/camera/image_raw', Image, handle_image_cb)
# tss = message_filters.ApproximateTimeSynchronizer([Subscriber('/camera/rgb/image_raw', Image),Subscriber("/camera/depth/image_raw", Image)],5,0.1)
# 		# self.camera_sub = rospy.Subscriber('/top_camera/camera/image_raw', Image, self._handle_image_cb)
# 		# tss = TimeSynchronizer(message_filters.Subscriber("/camera/rgb/image_raw", Image), message_filters.Subscriber("/camera/depth/image_raw", Image))
# tss.registerCallback(handle_image_cb)
cube_pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, handle_model_states_cb)

while not rospy.is_shutdown():
    dr.record()