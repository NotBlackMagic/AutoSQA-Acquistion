import numpy as np
import serial
import struct
import time

from enum import IntEnum

class AcquisitionController:
	class AirSensor:
		# Class that holds all values sampled by the Air Sensor
		class AirSensorFields:
			def __init__(self, temperature, humidity, pressure, gas):
				self.temperature = temperature
				self.humidity = humidity
				self.pressure = pressure
				self.gas = gas

		def __init__(self, serial_port):
			self.serial_port = serial_port	
			self.sensor_values = self.AirSensorFields(0.0, 0.0, 0.0, 0.0)
			
		def Acquisition(self):
			if self.serial_port == None or self.serial_port.isOpen() == False:
				return None
			
			# Request Air Sensor data
			cmd = "A;"
			self.serial_port.write(cmd.encode())
			# Wait for Air Sensor Data and parse it, values come as "T: %.2f, H: %.2f, P: %.2f, R: %.2f \n"
			string = self.serial_port.readline().decode("utf-8")
			string = string.replace(",", "")
			parts = string.split(" ")
			
			self.sensor_values.temperature = float(parts[1])
			self.sensor_values.humidity = float(parts[3])
			self.sensor_values.pressure = float(parts[5])
			self.sensor_values.gas = float(parts[7])
			return self.sensor_values
			

	class SoilSensor:
		# Class that holds all values sampled by the Air Sensor
		class SoilSensorFields:
			def __init__(self, ec, pH, force, success):
				self.ec = ec
				self.pH = pH
				self.force = force
				self.success = success

		def __init__(self, actuator_serial, acq_serial):
			self.actuator_serial = actuator_serial
			self.acq_serial = acq_serial
			self.sensor_values = self.SoilSensorFields(0.0, 0.0, 0.0, False)

		# Performs homing of the soil sensor actuator, blocking until complete
		def SoilSensorActuatorHoming(self):
			if self.actuator_serial == None or self.actuator_serial.isOpen() == False:
				return False
			
			cmd = "H;"
			self.actuator_serial.write(cmd.encode())
			while True:
				# Wait for homing complete
				cmd = "I;"
				self.actuator_serial.write(cmd.encode())
				string = self.actuator_serial.readline().decode("utf-8")
				parts = string.split(";")
				homing = int(parts[0])
				position = int(parts[1])
				max_force = int(parts[2])
				max_force_exceeded = int(parts[3])
				if homing == 0:
					break
				time.sleep(0.1)

			return True

		# Performs a movement of the soil sensor actuator, blocking until complete
		def SoilSensorActuatorMove(self, distance):
			if self.actuator_serial == None or self.actuator_serial.isOpen() == False:
				return False
			
			# Get current actuator position
			cmd = "P;"
			self.actuator_serial.write(cmd.encode())
			string = self.actuator_serial.readline().decode("utf-8")
			position = int(string)

			# Movement commands are relative
			target_position = distance + position
			if target_position > 85 or target_position < 0:
				return False

			# Check movement direction and send appropriate command
			if distance < 0:
				cmd = "U%d;" % (-distance)
				self.actuator_serial.write(cmd.encode())
			else:
				cmd = "D%d;" % (distance)
				self.actuator_serial.write(cmd.encode())

			# Wait for movement complete aka position reached
			while True:
				cmd = "I;"
				self.actuator_serial.write(cmd.encode())
				string = self.actuator_serial.readline().decode("utf-8")
				parts = string.split(";")
				homing = int(parts[0])
				position = int(parts[1])
				max_force = int(parts[2])
				max_force_exceeded = int(parts[3])
				if position == target_position or max_force_exceeded == 1:
					break
				time.sleep(1)

			if position == target_position:
				return True
			else:
				return False
			
		def Acquisition(self):
			if self.actuator_serial == None or self.actuator_serial.isOpen() == False or self.acq_serial == None or self.acq_serial.isOpen() == False:
				return None
			
			# Soil Sensor Acquisition requires a few steps
			# Step 1: Makes sure soil sensor actuator is homed
			success = self.SoilSensorActuatorHoming()
			if success == False:
				# Failed to home, abort
				self.sensor_values.success = False
				return self.sensor_values

			# Step 2: Move actuator to desired position aka down 50 mm
			success = self.SoilSensorActuatorMove(50)

			# Step 3: Get maximum force required/used
			cmd = "I;"
			self.actuator_serial.write(cmd.encode())
			string = self.actuator_serial.readline().decode("utf-8")
			parts = string.split(";")
			homing = int(parts[0])
			position = int(parts[1])
			max_force = int(parts[2])
			max_force_exceeded = int(parts[3])
			# Save values
			self.sensor_values.force = max_force
			self.sensor_values.success = success
			
			# Step 4: Acquire values from soil sensor
			# We could do this only if movement (Step 3) was successful but just do it anyways. Movement success is reflected in .success variable value
			# Request Soil Sensor data
			cmd = "S;"
			self.acq_serial.write(cmd.encode())
			# Wait for Air Sensor Data and parse it, values come as "H: %.2f, P: %.2f \n"
			string = self.acq_serial.readline().decode("utf-8")
			string = string.replace(",", "")
			parts = string.split(" ")
			
			self.sensor_values.ec = float(parts[1])
			self.sensor_values.pH = float(parts[3])

			# Step 5: Move actuator back to home position
			success = self.SoilSensorActuatorHoming()
			
			return self.sensor_values
		
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
	
# try:
# 	acq_serial = serial.Serial(
# 		port = "COM3",
# 		baudrate = 115200,
# 		bytesize = serial.EIGHTBITS,
# 		stopbits = serial.STOPBITS_ONE,
# 		parity = serial.PARITY_NONE)
# except serial.SerialException as e:
# 	print("could not open serial port '{}': {}".format("COM4", e))
# 	acq_serial = None

# try:
# 	actuator_serial = serial.Serial(
# 		port = "COM13",
# 		baudrate = 115200,
# 		bytesize = serial.EIGHTBITS,
# 		stopbits = serial.STOPBITS_ONE,
# 		parity = serial.PARITY_NONE)
# except serial.SerialException as e:
# 	print("could not open serial port '{}': {}".format("COM13", e))
# 	actuator_serial = None

# acq_air = AcquisitionController.AirSensor(acq_serial)
# acq_soil = AcquisitionController.SoilSensor(actuator_serial, acq_serial)
# while True:
# 	air = acq_air.Acquisition()
# 	print("Temperature: %.2f; Humidity: %.2f; Pressure: %.2f; Gas: %.2f;" % (air.temperature, air.humidity, air.pressure, air.gas))
# 	soil = acq_soil.Acquisition()
# 	print("Humidity: %.2f; pH: %.2f;" % (soil.ec, soil.pH))
# 	time.sleep(1)