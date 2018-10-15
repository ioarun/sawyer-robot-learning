# Script to help use Novint Falcon Joystick
# device with the Sawyer robot to collect
# demonstrations.
import rospy
from sensor_msgs.msg import Joy

class NovintFalcon(object):
	def __init__(self):
		sub = rospy.Subscriber('/falcon/joystick', Joy, self._on_msg)
		self._value_x = 0.0
		self._value_y = 0.0
		self._value_z = 0.0
		self._current_value_z = 0.0
		self._button = 0

	def _on_msg(self, msg):
		self._value_x = msg.axes[0]
		self._value_y = msg.axes[1]
		self._value_z = msg.axes[2]
		self._button = msg.buttons[0]


