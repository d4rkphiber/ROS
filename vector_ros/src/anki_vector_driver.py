#!/usr/bin/python3
# -*- encoding: utf-8 -*-


import sys
#sys.path.insert(0,"/home/admin/.local/lib/python3.6/site-packages")
import rospy
import tf2_ros
import tf
import cv2
import yaml
#from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy
import numpy as np
import anki_vector
from anki_vector.util import radians

from transformations import quaternion_from_euler
#from camera_info_manager import CameraInfoManager

# ROS msgs
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
	Twist,
	TransformStamped
)
from std_msgs.msg import (
	String,
	Float64,
	ColorRGBA,
)
from sensor_msgs.msg import (
	Image,
	CameraInfo,
	BatteryState,
	Imu,
	JointState,
	Range
)



class TransformBroadcaster(object):
	"""
	:class:`TransformBroadcaster` is a convenient way to send transformation updates on the ``"/tf"`` message topic.
	"""

	def __init__(self, queue_size=100):
		self.pub_tf = rospy.Publisher("/tf", TFMessage, queue_size=queue_size)

	def send_transform(self, translation, rotation, time, child, parent):
		"""
		:param translation: the translation of the transformation as a tuple (x, y, z)
		:param rotation: the rotation of the transformation as a tuple (x, y, z, w)
		:param time: the time of the transformation, as a rospy.Time()
		:param child: child frame in tf, string
		:param parent: parent frame in tf, string
		Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
		"""

		t = TransformStamped()
		t.header.frame_id = parent
		t.header.stamp = time
		t.child_frame_id = child
		t.transform.translation.x = translation[0]
		t.transform.translation.y = translation[1]
		t.transform.translation.z = translation[2]

		t.transform.rotation.x = rotation[0]
		t.transform.rotation.y = rotation[1]
		t.transform.rotation.z = rotation[2]
		t.transform.rotation.w = rotation[3]

		self.send_transform_message(t)

	def send_transform_message(self, transform):
		"""
		:param transform: geometry_msgs.msg.TransformStamped
		Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
		"""
		tfm = TFMessage([transform])
		self.pub_tf.publish(tfm)



def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by
    rosrun camera_calibration cameracalibrator.py) into a
    sensor_msgs/CameraInfo msg.

    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data

    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
	
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
      calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    camera_info_msg.header.frame_id = 'camera_info'
    return camera_info_msg


class VectorRos(object):

	def __init__(self, vec):
	  
		
		# vars
		self._vector = vec
		self._vector.camera.init_camera_feed()
		self._lin_vel = .0
		self._ang_vel = .0
		self._cmd_lin_vel = .0
		self._cmd_ang_vel = .0
		self._last_pose = self._vector.pose
		self._wheel_vel = (0, 0)
		self._optical_frame_orientation = quaternion_from_euler(-np.pi/2., .0, -np.pi/2.)
		#self._camera_info_manager = CameraInfoManager('vector_camera', namespace='/vector_camera')

		self._tfb = TransformBroadcaster()

		# params
		self._odom_frame = rospy.get_param('~odom_frame', 'odom')
		self._footprint_frame = rospy.get_param('~footprint_frame', 'base_footprint')
		self._base_frame = rospy.get_param('~base_frame', 'base_link')
		self._head_frame = rospy.get_param('~head_frame', 'head_link')
		self._camera_frame = rospy.get_param('~camera_frame', 'camera_link_optical')
		self._tof_sensor_frame = rospy.get_param('~tof_frame', 'tof_link')

		#self._camera_optical_frame = rospy.get_param('~camera_optical_frame', 'vector_camera')
		#camera_info_url = rospy.get_param('~camera_info_url', 'file:///home/ubuntu/.ros/vector.yaml')

		# pubs
		self._joint_state_pub = rospy.Publisher('joint_states', JointState, queue_size=1)
		self._odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
		self._imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
		self._battery_pub = rospy.Publisher('battery', BatteryState, queue_size=1)
		self._tof_sensor_pub= rospy.Publisher('tof_sensor', Range, queue_size=10)
		
		self._video_pub= rospy.Publisher('/vector_camera/image', Image, queue_size=10)
		#self._camera_info_pub = rospy.Publisher('/vector_camera/camera_info', CameraInfo, queue_size=10)
		self._video_info_pub = rospy.Publisher('/vector_camera/camera_info', CameraInfo, queue_size=10)
		filename="/home/ubuntu/.ros/vector.yaml"
		self._camera_info_msg = yaml_to_CameraInfo(filename)

		

		# subs
		#self._backpack_led_sub = rospy.Subscriber('backpack_led', ColorRGBA, self._set_backpack_led, queue_size=1)
		self._twist_sub = rospy.Subscriber('cmd_vel', Twist, self._twist_callback, queue_size=1)
		self._say_sub = rospy.Subscriber('say', String, self._say_callback, queue_size=1)
		#self._head_sub = rospy.Subscriber('head_angle', Float64, self._move_head, queue_size=1)
		#self._lift_sub = rospy.Subscriber('lift_height', Float64, self._move_lift, queue_size=1)
		
		self._listener = tf.TransformListener()

		
	def _publish_battery(self):
	   
	   
		# only publish if we have a subscriber
		if self._battery_pub.get_num_connections() == 0:
			return

		battery_state = self._vector.get_battery_state()
		battery = BatteryState()
		volts = battery_state.battery_volts
		battery.header.stamp = rospy.Time.now()
		battery.voltage = volts
		battery.present = True
		if battery_state.is_on_charger_platform:  # is_charging always return False
			battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
		else:
			battery.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING
		
		self._battery_pub.publish(battery)      
		
		

	def _publish_imu(self):
		"""
		Publish inertia data as Imu message.
		"""
		# only publish if we have a subscriber
		if self._imu_pub.get_num_connections() == 0:
			return

		imu = Imu()
		imu.header.stamp = rospy.Time.now()
		imu.header.frame_id = self._base_frame
		imu.orientation.w = self._vector.pose.rotation.q0
		imu.orientation.x = self._vector.pose.rotation.q1
		imu.orientation.y = self._vector.pose.rotation.q2
		imu.orientation.z = self._vector.pose.rotation.q3
		imu.angular_velocity.x = self._vector.gyro.x
		imu.angular_velocity.y = self._vector.gyro.y
		imu.angular_velocity.z = self._vector.gyro.z
		imu.linear_acceleration.x = self._vector.accel.x * 0.001
		imu.linear_acceleration.y = self._vector.accel.y * 0.001
		imu.linear_acceleration.z = self._vector.accel.z * 0.001
		self._imu_pub.publish(imu)

	def _publish_odometry(self):
		"""
		Publish current pose as Odometry message.
		"""
		# only publish if we have a subscriber
		if self._odom_pub.get_num_connections() == 0:
			return

		now = rospy.Time.now()
		odom = Odometry()
		odom.header.frame_id = self._odom_frame
		odom.header.stamp = now
		odom.child_frame_id = self._footprint_frame
		odom.pose.pose.position.x = self._vector.pose.position.x * 0.001
		odom.pose.pose.position.y = self._vector.pose.position.y * 0.001
		odom.pose.pose.position.z = self._vector.pose.position.z * 0.001
		q = quaternion_from_euler(.0, .0, self._vector.pose_angle_rad)
		
		# reihenfolge falsch?
		odom.pose.pose.orientation.w = q[0]
		odom.pose.pose.orientation.x = q[1]
		odom.pose.pose.orientation.y = q[2]
		odom.pose.pose.orientation.z = q[3]
		odom.pose.covariance = np.diag([1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e-1]).ravel()
		odom.twist.twist.linear.x = self._lin_vel
		odom.twist.twist.angular.z = self._ang_vel
		odom.twist.covariance = np.diag([1e-2, 1e3, 1e3, 1e3, 1e3, 1e-2]).ravel()
		self._odom_pub.publish(odom)

	def _publish_video(self):
		if self._vector.camera.image_streaming_enabled():
		#if robot.camera.image_streaming_enabled():
			image = cv2.cvtColor(np.array(self._vector.camera.latest_image.raw_image),cv2.COLOR_RGB2BGR)
			msg_frame = CvBridge().cv2_to_imgmsg(image, "bgr8")
			now = rospy.Time.now()
			msg_frame.header.frame_id = self._camera_frame
			msg_frame.header.stamp =now
			self._camera_info_msg.header.stamp =now
			self._video_pub.publish(msg_frame)
			self._video_info_pub.publish(self._camera_info_msg)
			#camera_info = self._camera_info_manager.getCameraInfo()
			#self._camera_info_pub.publish(camera_info)


	def _twist_callback(self, cmd):
		"""
		Set commanded velocities from Twist message.
		The commands are actually send/set during run loop, so delay
		is in worst case up to 1 / update_rate seconds.
		:type   cmd:    Twist
		:param  cmd:    The commanded velocities.
		"""
		# compute differential wheel speed
		axle_length = 0.07  # 7cm
		self._cmd_lin_vel = cmd.linear.x
		self._cmd_ang_vel = cmd.angular.z
		rv = self._cmd_lin_vel + (self._cmd_ang_vel * axle_length * 0.5)
		lv = self._cmd_lin_vel - (self._cmd_ang_vel * axle_length * 0.5)
		self._wheel_vel = (lv*1000., rv*1000.)  # convert to mm / s

	def _publish_tof_sensor(self):
		#sensor_msgs/Range
		#
		range = Range()
		now = rospy.Time.now()
		range.header.frame_id = self._tof_sensor_frame
		range.header.stamp = now
		proximity_data= self._vector.proximity.last_sensor_reading
		range.range=proximity_data.distance.distance_mm
		range.min_range=0.3
		range.max_range=1.5
		range.radiation_type=1

		if proximity_data is not None:
			#print (proximity_data.distance.distance_mm)
			self._tof_sensor_pub.publish(range)

	def _say_callback(self,say):
		print (say)
		self._vector.behavior.say_text(say.data)


	def _publish_objects(self):
		"""
		Publish detected object as transforms between odom_frame and object_frame.
		"""

		for obj in self._vector.world.visible_objects:
			now = rospy.Time.now()
			#print (obj)
			print (obj.descriptive_name, ':', obj.pose)
			x = obj.pose.position.x * 0.001
			y = obj.pose.position.y * 0.001
			z = obj.pose.position.z * 0.001
			q = (obj.pose.rotation.q1, obj.pose.rotation.q2, obj.pose.rotation.q3, obj.pose.rotation.q0)

			self._tfb.send_transform(
				(x, y, z), q, now, 'cube_' + str(obj.object_id), self._odom_frame
			)


	def _publish_tf(self, update_rate):
		"""
		Broadcast current transformations and update
		measured velocities for odometry twist.
		Published transforms:
		odom_frame -> footprint_frame
		footprint_frame -> base_frame
		base_frame -> head_frame
		head_frame -> camera_frame
		camera_frame -> camera_optical_frame
		"""
		now = rospy.Time.now()
		x = self._vector.pose.position.x * 0.001
		y = self._vector.pose.position.y * 0.001
		z = self._vector.pose.position.z * 0.001

		# compute current linear and angular velocity from pose change
		# Note: Sign for linear velocity is taken from commanded velocities!
		# Note: The angular velocity can also be taken from gyroscopes!
		
		
	
		#delta_pose = self._last_pose - self._vector.pose
		delta_pose_x = self._last_pose.position.x - self._vector.pose.position.x
		delta_pose_y = self._last_pose.position.y - self._vector.pose.position.y
		delta_pose_z = self._last_pose.position.z - self._vector.pose.position.z
		delta_pose_angle_z = self._last_pose.rotation.angle_z.radians -  self._vector.pose.rotation.angle_z.radians
		dist = np.sqrt(delta_pose_x**2
					   + delta_pose_y**2
					   + delta_pose_z**2) / 100.0
		self._lin_vel = dist * update_rate * np.sign(self._cmd_lin_vel)
		self._ang_vel = -delta_pose_angle_z * update_rate

		# publish odom_frame -> footprint_frame
		q = quaternion_from_euler(.0, .0, self._vector.pose_angle_rad)
		self._tfb.send_transform(
			(x, y, 0.0), q, now, self._footprint_frame, self._odom_frame)

		# publish footprint_frame -> base_frame
		q = quaternion_from_euler(.0, -self._vector.pose_pitch_rad, .0)
		self._tfb.send_transform(
			(0.0, 0.0, 0.02), q, now, self._base_frame, self._footprint_frame)

		# publish base_frame -> head_frame
		q = quaternion_from_euler(.0, -self._vector.head_angle_rad, .0)
		self._tfb.send_transform(
			(0.02, 0.0, 0.05), q, now, self._head_frame, self._base_frame)

		# publish head_frame -> camera_frame
		#hier drehen wir den Fram, damit er in rviz richtg dargestellt wird.
		#allerding ist die Achse immer noch verkehrt
		q = quaternion_from_euler(-np.pi/2., .0, -np.pi/2.)
		self._tfb.send_transform((0.025, 0.0, -0.015), q, now, self._camera_frame, self._head_frame)
		#self._tfb.send_transform((0.025, 0.0, -0.015), (0.0, 0.0, 0.0, 1.0), now, self._camera_frame, self._head_frame)

		# publish camera_frame -> camera_optical_frame
		#q = self._optical_frame_orientation
		#self._tfb.send_transform(
		#	(0.0, 0.0, 0.0), q, now, self._camera_optical_frame, self._camera_frame)

		# store last pose
		self._last_pose = deepcopy(self._vector.pose)

		#try:
		#	(trans, rot) = self._listener.lookupTransform('camera_link', 'robot1', rospy.Time(0))
		#	print (trans)
		#except (tf.LookupException, tf.ConnectivityException):
		#	print ("Warn")

		
		# experimental : Transform base_frame and publish as an extra frame for rviz
		try:
			#q = self._optical_frame_orientation
			self._tfb.send_transform((5.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), now, 'tag000', self._base_frame)
		except:
			rospy.logwarn("no frame")

		


		


	def run(self, update_rate=10):
		r = rospy.Rate(update_rate)
		while not rospy.is_shutdown():
			self._publish_tf(update_rate)
			self._publish_objects()
			#self._publish_joint_state()
			self._publish_imu()
			self._publish_battery()
			self._publish_odometry()
			self._publish_video()
			self._publish_tof_sensor()
			#self._video_info_pub.publish
			#self._publish_diagnostics()
			# send message repeatedly to avoid idle mode.
			# This might cause low battery soon
			# TODO improve this!
			# (*self._wheel_vel)  Stern bedeutet hier:When calling a function, 
			# the * operator can be used to unpack an iterable into the arguments in the function call:
			self._vector.motors.set_wheel_motors(*self._wheel_vel)
			# sleep
			
			r.sleep()
		
	   

if __name__ == '__main__':
	rospy.init_node('anki_vector_driver')

	#
	# es gibt zwei Möglichkeiten, eine Verbindung zu starten
	#
	# direkt über ein Objekt und die Methoden connect zum Aufbau und disconnect zum Beenden
	#
	# with legt ein Objekt an, bei dem am Start gleich eine Verbindung aufgebaut wird und bei Ende automatisch beendet wird
	# https://developer.anki.com/vector/docs/generated/anki_vector.robot.html
	
	# robot = anki_vector.AsyncRobot()
	# Connect to Vector
	# robot.connect()
	# Run your commands
	# robot.anim.play_animation("anim_turn_left_01").result()
	# Disconnect from Vector
	# robot.disconnect()
	#
	# Update AsyncRobot funktioniert hier leider nicht, 
	# AttributeError: 'Future' object has no attribute 'battery_volts'
	#
	#
	# fill anki_vector.Robot() with options at object instatiating
	# show_viewer=True

	with anki_vector.Robot(show_viewer=True, show_3d_viewer=True) as robot:
		vec_ros = VectorRos(robot)
		vec_ros.run()
		
	
	
	
