import math
import numpy as np
import serial
import socket
import struct

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node

from acquisition.acquisition_controller import AcquisitionController
from acquisition.acquisition_controller import SensorData

# ROS messages
from nav_msgs.msg import Odometry

class AcquisitionHub(Node):
	def __init__(self):
		super().__init__("AcquisitionHub")

		# Set used parameters
		param_descriptor = ParameterDescriptor(description = "Sets the subscribed Odometry topic name.")
		self.declare_parameter("odom_topic", "/wheel/odometry", param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the acquisition server address.")
		self.declare_parameter("server_address", "192.168.1.83", param_descriptor)
		param_descriptor = ParameterDescriptor(description = "Sets the acquisition server port.")
		self.declare_parameter("server_port", 25003, param_descriptor)

		# Load parameters
		odom_topic = self.get_parameter("odom_topic").value
		server_address = self.get_parameter("server_address").value
		server_port = self.get_parameter("server_port").value
		sampling_loop_rate = 1

		# Init subscriber variables
		self.current_position = np.array([0.0, 0.0, 0.0])
		self.current_orientation = np.array([0.0, 0.0, 0.0, 0.0])
		self.current_velocity_linear = np.array([0.0, 0.0, 0.0])
		self.current_velocity_angular = np.array([0.0, 0.0, 0.0])

		# Start ROS Subscribers
		self.odom_subscriber = self.create_subscription(
			Odometry,
			odom_topic,
			self.odom_callback,
			10)
		
		# Connect to acquisition server
		self.serverAddressPort = (server_address, server_port)
		self.udpSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

		# Connect sensors through acquisition hub
		try:
			acq_serial = serial.Serial(
			port = "COM3",
			baudrate = 115200,
			bytesize = serial.EIGHTBITS,
			stopbits = serial.STOPBITS_ONE,
			parity = serial.PARITY_NONE)
		except serial.SerialException as e:
			print("could not open serial port '{}': {}".format("COM4", e))
			acq_serial = None
		try:
			actuator_serial = serial.Serial(
			port = "COM13",
			baudrate = 115200,
			bytesize = serial.EIGHTBITS,
			stopbits = serial.STOPBITS_ONE,
			parity = serial.PARITY_NONE)
		except serial.SerialException as e:
			print("could not open serial port '{}': {}".format("COM13", e))
			actuator_serial = None

		self.acq_air = AcquisitionController.AirSensor(acq_serial)
		self.acq_soil = AcquisitionController.SoilSensor(actuator_serial, acq_serial)

		self.sensor_data = SensorData()
		
		# Start sampling loop update timer
		timer_period = 1.0 / sampling_loop_rate  # seconds
		self.timer = self.create_timer(timer_period, self.sampling_callback)
		self.pid_dt = timer_period
		
	# Callback function for new Odometry message reception
	def odom_callback(self, msg):
		self.current_position[0] = msg.pose.pose.position.x
		self.current_position[1] = msg.pose.pose.position.y
		self.current_position[2] = msg.pose.pose.position.z
		self.current_orientation[0] = msg.pose.pose.orientation.x
		self.current_orientation[1] = msg.pose.pose.orientation.y
		self.current_orientation[2] = msg.pose.pose.orientation.z
		self.current_orientation[3] = msg.pose.pose.orientation.w
		self.current_velocity_linear[0] = msg.twist.twist.linear.x
		self.current_velocity_linear[1] = msg.twist.twist.linear.y
		self.current_velocity_linear[2] = msg.twist.twist.linear.z
		self.current_velocity_angular[0] = msg.twist.twist.angular.x
		self.current_velocity_angular[1] = msg.twist.twist.angular.y
		self.current_velocity_angular[2] = msg.twist.twist.angular.z

	def sampling_callback(self):
		# Set sample coordinate
		# self.sensor_data.local_coordinate_x = self.current_position[0]
		# self.sensor_data.local_coordinate_y = self.current_position[1]
		# self.sensor_data.local_coordinate_z = self.current_position[2]

		# Generate new coordinate, for testing purpose only
		self.sensor_data.local_coordinate_x -= 2
		self.sensor_data.local_coordinate_y = 0
		self.sensor_data.local_coordinate_z = 0

		# Get new values from sensors
		# Air Sensor values/acquisitions
		air_sensor_data = self.acq_air.Acquisition()
		if air_sensor_data != None:
			print("Temperature: %.2f; Humidity: %.2f; Pressure: %.2f; Gas: %.2f;" % (air_sensor_data.temperature, air_sensor_data.humidity, air_sensor_data.pressure, air_sensor_data.gas))

			# Add air temperature (in ºC)
			sensor_type = SensorData.SensorDataTypes.Air_Temperature
			self.sensor_data.sensor_data_types += (int)(1 << sensor_type)
			# sensor_data.sensor_data[(int)(sensor_type)] = 25.0 + random.uniform(-10.0, 10.0)
			self.sensor_data.sensor_data[(int)(sensor_type)] = air_sensor_data.temperature
			# Add air humidity (in %)
			sensor_type = SensorData.SensorDataTypes.Air_Humidity
			self.sensor_data.sensor_data_types += (int)(1 << sensor_type)
			# sensor_data.sensor_data[(int)(sensor_type)] = 60.0 + random.uniform(-15.0, 15.0)
			self.sensor_data.sensor_data[(int)(sensor_type)] = air_sensor_data.humidity
			# Add air pressure (in hPa)
			sensor_type = SensorData.SensorDataTypes.Air_Pressure
			self.sensor_data.sensor_data_types += (int)(1 << sensor_type)
			# sensor_data.sensor_data[(int)(sensor_type)] = 1000.0 + random.uniform(-100.0, 100.0)
			self.sensor_data.sensor_data[(int)(sensor_type)] = air_sensor_data.pressure

		# Soil Sensor values/acquisitions
		soil_sensor_data = self.acq_soil.Acquisition()
		if soil_sensor_data != None:
			print("Success: %d; Force: %.2f; EC: %.2f; pH: %.2f;" % (soil_sensor_data.success, soil_sensor_data.force, soil_sensor_data.ec, soil_sensor_data.pH))

			# Add soil force value
			sensor_type = SensorData.SensorDataTypes.Soil_Hardness
			self.sensor_data.sensor_data_types += (int)(1 << sensor_type)
			self.sensor_data.sensor_data[(int)(sensor_type)] = soil_sensor_data.force
			# Add soil EC value
			sensor_type = SensorData.SensorDataTypes.Soil_EC
			self.sensor_data.sensor_data_types += (int)(1 << sensor_type)
			self.sensor_data.sensor_data[(int)(sensor_type)] = soil_sensor_data.ec
			# Add soil pH value
			sensor_type = SensorData.SensorDataTypes.Soil_pH
			self.sensor_data.sensor_data_types += (int)(1 << sensor_type)
			self.sensor_data.sensor_data[(int)(sensor_type)] = soil_sensor_data.pH

		data_struct = self.sensor_data.encode()
		self.udpSocket.sendto(data_struct, self.serverAddressPort)

def main(args=None):
	rclpy.init(args=args)

	print("Starting Acquisition Hub...\n")

	acquisition_hub = AcquisitionHub()

	rclpy.spin(acquisition_hub)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	acquisition_hub.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()