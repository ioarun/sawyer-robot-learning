from sawyer_demonstration.srv import *
import rospy
from std_srvs.srv import Empty

def handle_start_recording(req):
    print "returning start recording flag"
    return StartRecordingResponse()

def start_recording_server():
	rospy.init_node('start_recording')
	s = rospy.Service('start_recording', StartRecording, handle_start_recording)
	print "Ready to start recording"
	rospy.spin()

if __name__ == "__main__":
    start_recording_server()