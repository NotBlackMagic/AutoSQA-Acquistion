from enum import IntEnum
import numpy as np
import random
import socket
import struct
import time

from acquisitionHub import AcquisitionHub

class SensorData:
	class SensorDataTypes(IntEnum):
		Air_Temperature = 0
		Air_Humidity = 1
		Air_Pressure = 2
		Magnetic_Field = 3
		Soil_Temperature = 4
		Soil_Humidity = 5
		Soil_EC = 6
		Soil_pH = 7
		Soil_Salinity = 8
		Soil_Hardness = 9

	def __init__(self):
		# Header Data
		self.timestamp = 0.0
		self.local_coordinate_x = 0.0
		self.local_coordinate_y = 0.0
		self.local_coordinate_z = 0.0
		self.sensor_data_types = 0

		# Sensor Data
		self.sensor_data = np.zeros(10)

	def encode(self):
		# Pack header
		buffer = struct.pack(
			"ffffi",
			self.timestamp,
			self.local_coordinate_x,
			self.local_coordinate_y,
			self.local_coordinate_z,
			self.sensor_data_types)
		# Pack sensor data
		for s in range(9):
			enabled = (self.sensor_data_types >> s) & 1
			if enabled == 1:
				buffer += struct.pack("f", self.sensor_data[s])
		
		return buffer
	

serverAddressPort = ("127.0.0.1", 20001)

# Create a UDP socket at client side
udpSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Start sensors through acquisition hub
acq_air = AcquisitionHub.AirSensor("COM4", 921600)
acq_soil = AcquisitionHub.SoilSensor("COM3", 115200)

sensor_data = SensorData()

state = 0
# Send to server using created UDP socket
while True:
	# Add timestamp
	sensor_data.timestamp = time.time()

	# Add local coordinates
	if state == 0:
		sensor_data.local_coordinate_x -= 0.5
		if sensor_data.local_coordinate_x <= -80:
			print("Change State to 1")
			state = 1
	elif state == 1:
		sensor_data.local_coordinate_y += 0.5
		if sensor_data.local_coordinate_y >= 30:
			print("Change State to 2")
			state = 2
	elif state == 2:
		sensor_data.local_coordinate_x += 0.5
		if sensor_data.local_coordinate_x >= 0:
			print("Change State to 3")
			state = 3
	elif state == 3:
		sensor_data.local_coordinate_y -= 0.5
		if sensor_data.local_coordinate_y <= 0:
			print("Change State to 0")
			state = 0

	# Clear sensor data types
	sensor_data.sensor_data_types = 0

	# Get new values from sensors
	# Air Sensor values/acquisitions
	air_sensor_data = acq_air.Acquisition()
	if air_sensor_data != None:
		print("Temperature: %.2f; Humidity: %.2f; Pressure: %.2f; Gas: %.2f;" % (air_sensor_data.temperature, air_sensor_data.humidity, air_sensor_data.pressure, air_sensor_data.gas))

		# Add air temperature (in ºC)
		sensor_type = SensorData.SensorDataTypes.Air_Temperature
		sensor_data.sensor_data_types += (int)(1 << sensor_type)
		# sensor_data.sensor_data[(int)(sensor_type)] = 25.0 + random.uniform(-10.0, 10.0)
		sensor_data.sensor_data[(int)(sensor_type)] = air_sensor_data.temperature
		# Add air humidity (in %)
		sensor_type = SensorData.SensorDataTypes.Air_Humidity
		sensor_data.sensor_data_types += (int)(1 << sensor_type)
		# sensor_data.sensor_data[(int)(sensor_type)] = 60.0 + random.uniform(-15.0, 15.0)
		sensor_data.sensor_data[(int)(sensor_type)] = air_sensor_data.humidity
		# Add air pressure (in hPa)
		sensor_type = SensorData.SensorDataTypes.Air_Pressure
		sensor_data.sensor_data_types += (int)(1 << sensor_type)
		# sensor_data.sensor_data[(int)(sensor_type)] = 1000.0 + random.uniform(-100.0, 100.0)
		sensor_data.sensor_data[(int)(sensor_type)] = air_sensor_data.pressure

	# Soil Sensor values/acquisitions
	soil_sensor_data = acq_soil.Acquisition()
	if soil_sensor_data != None:
		print("Success: %d; Force: %.2f; EC: %.2f; pH: %.2f;" % (soil_sensor_data.success, soil_sensor_data.force, soil_sensor_data.ec, soil_sensor_data.pH))

		# Add soil force value
		sensor_type = SensorData.SensorDataTypes.Soil_Hardness
		sensor_data.sensor_data_types += (int)(1 << sensor_type)
		sensor_data.sensor_data[(int)(sensor_type)] = soil_sensor_data.force
		# Add soil EC value
		sensor_type = SensorData.SensorDataTypes.Soil_EC
		sensor_data.sensor_data_types += (int)(1 << sensor_type)
		sensor_data.sensor_data[(int)(sensor_type)] = soil_sensor_data.ec
		# Add soil pH value
		sensor_type = SensorData.SensorDataTypes.Soil_pH
		sensor_data.sensor_data_types += (int)(1 << sensor_type)
		sensor_data.sensor_data[(int)(sensor_type)] = soil_sensor_data.pH

	data_struct = sensor_data.encode()
	udpSocket.sendto(data_struct, serverAddressPort)

	# data = "Hello Python UDP here!"
	# udpSocket.sendto(str.encode(data), serverAddressPort)
	# response = udpSocket.recvfrom(1024)
	# msg = "Message from Server {}".format(response[0])
	# print(msg)

	time.sleep(1)
