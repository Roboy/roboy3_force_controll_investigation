from os import read
import re
import rospy
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.msg import MotorState
from roboy_middleware_msgs.srv import ControlMode
from rospy.impl.tcpros_service import Service
from std_msgs.msg import String

from Phidget22.Phidget import *
from Phidget22.Devices.VoltageRatioInput import *

import numpy as np
import csv
from time import time, sleep

class ConnectionError(Exception):
	"""Raised when connection to load cell channels fails."""
	def __init__(self, message):
		self.message = message


class LoadCell(VoltageRatioInput):
	"""This class handles a single load cell channel.
	
	"""
	def __init__(self, conf):
		"""
		Args:
			conf (dict): Dictionary with initial channel configuration:
							'tendon_id' 	(int): Tendon id to which the cell is attached.
							'cal_offset'  (float): Calibration offset value of the load cell.
							'cal_factor'  (float): Calibration factor value of the load cell.
							'serial' 		(int): Serial number of the phidget to which the cell is connected.
							'channel' 		(int): Number of the phidget channel to which the cell is connected.
		
		"""
		super().__init__()
		self.id = conf['tendon_id']
		self.cal_offset = conf['cal_offset']
		self.cal_factor = conf['cal_factor']
		self.setDeviceSerialNumber(conf['serial'])
		self.setChannel(conf['channel'])

	@property
	def ready(self):
		return self.cal_offset is not None and self.cal_factor is not None

	def getAddress(self):
		return f"{self.getDeviceSerialNumber()}/{self.getChannel()}"

	def openChannel(self):
		"""Opens and attaches to phidget channel.
		
		Args:
			-
		Returns:
		   	-
		"""
		if not self.getAttached():
			try:
				self.openWaitForAttachment(Phidget.DEFAULT_TIMEOUT)
			except PhidgetException as e:
				raise ConnectionError(e.details)

	def closeChannel(self):
		"""Closes phidget channel.
		
		Args:
			-
		Returns:
		   	-
		"""
		self.close()

	def readForce(self):
		"""Reads force value from load cell in Newtons.
		
		Args:
			-
		Returns:
		   	float: Force value in Newtons.
		"""
		force = 0
		if self.ready:
			force = self.cal_factor * (self.getVoltageRatio() + self.cal_offset)
		return force * 9.81


MOTOR_NUM = 7
#CONVERSION_FACTOR = 0.00000651941

DISTANCE_STEPS = 0.01

WEIGHT = ""

readValuesLabel = [ "timestamp",
                    "pwm",
                    "readSetpoints",
                    "encoder0Vals",
                    "encoder1Vals",
                    "displacements",
                    "loadcellforce",
                    "current"]
readValues = []

readValuesAtDistanceMarks = []


writtenSetpoinits = []

loadcellforce = 0

def callback(data):
    #print(writtenSetpoinits[len(writtenSetpoinits)-1])
    loadcellforce = channels[0].readForce()
    readValues.append([rospy.get_rostime().nsecs,writtenSetpoinits[len(writtenSetpoinits)-1],data.setpoint[MOTOR_NUM],data.encoder0_pos[MOTOR_NUM],data.encoder1_pos[MOTOR_NUM],data.displacement[MOTOR_NUM],loadcellforce,data.current[MOTOR_NUM]])


phidget_serial = 585671
cal_offsets = [-1.0473654e-05]
cal_factors = [-7956.613617122953]

configuration = [{'tendon_id': i, 'cal_offset': o, 'cal_factor': f, 'serial': phidget_serial, 'channel': 1} for i, (o, f) in enumerate(zip(cal_offsets, cal_factors))]

def main():
    rospy.init_node("elbow_test")
	
    #service = rospy.ServiceProxy(' /roboy/pinky/middleware/ControlMode',ControlMode)
    pub = rospy.Publisher('/roboy/pinky/middleware/MotorCommand', MotorCommand, queue_size=1)
    sub = rospy.Subscriber('/roboy/pinky/middleware/MotorState',MotorState, callback)

    #service(3,[0],[MOTOR_NUM])

    setpoints = np.arange(0, 5, 1)
    cmd = MotorCommand()
    cmd.global_id = [MOTOR_NUM]

    rate = rospy.Rate(600)

    try:
        for s in setpoints:
            writtenSetpoinits.append(s)
            cmd.setpoint = [s]
            pub.publish(cmd)
            rate.sleep()

            while(len(readValues) > 1 and readValues[len(readValues)-1][3] != readValues[len(readValues)-2][3]):
                rate.sleep()
            print(s)
            print("Press any key to continue")
            input()


    except Exception as e:
        print(e.message,e.args)
        print("Save Data? (y/anything else)")
        if(input() != "y"):
            return

    with open("motorDataForceMapping-pwm-"+str(int(time()))+".csv", 'w') as f: 
        write = csv.writer(f) 
        write.writerow(readValuesLabel) 
        write.writerows(readValues)
    print("SAVED")
    print("Bitte Taste zum Ablassen drücken")
    input()
    cmd.setpoint = [0.0]
    pub.publish(cmd)
    rate.sleep()

if __name__ == '__main__':
    channels = []
    for i, conf in enumerate(configuration):
        new_channel = LoadCell(conf)
        try:
            new_channel.openChannel()
            new_channel.setDataInterval(10)
            channels.append(new_channel)
        except ConnectionError as e:
            print(f"Failed to open load cell {i}: {e.message}")   
    main()
    # Close your Phidgets once the program is done.
    for channel in channels:
        channel.closeChannel()
        print("closed")