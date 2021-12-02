from os import read
import re
import rospy
from roboy_middleware_msgs.msg import MotorCommand
from roboy_middleware_msgs.msg import MotorState
from std_msgs.msg import String

import numpy as np
import csv
from time import time

MOTOR_NUM = 7
CONVERSION_FACTOR = 0.00000651941

readValuesLabel = [ "timestamp",
                    "writtenSetpoinits",
                    "readSetpoints",
                    "encoder0Vals",
                    "encoder1Vals",
                    "displacements"]
readValues = []
writtenSetpoinits = []

def callback(data):
    print(writtenSetpoinits[len(writtenSetpoinits)-1])
    readValues.append([rospy.get_rostime().nsecs,writtenSetpoinits[len(writtenSetpoinits)-1],data.setpoint[MOTOR_NUM],data.encoder0_pos[MOTOR_NUM],data.encoder1_pos[MOTOR_NUM],data.displacement[MOTOR_NUM]])

def main():
    rospy.init_node("elbow_test")

    pub = rospy.Publisher('/roboy/pinky/middleware/MotorCommand', MotorCommand, queue_size=1)
    sub = rospy.Subscriber('/roboy/pinky/middleware/MotorState',MotorState, callback)

    setpoints = np.arange(0, 0.6, 0.00001)

    cmd = MotorCommand()
    cmd.global_id = [MOTOR_NUM]

    rate = rospy.Rate(600)

    for s in setpoints:
        writtenSetpoinits.append(s)
        cmd.setpoint = [s]
        pub.publish(cmd)
        rate.sleep()

        while(len(readValues) > 1 and readValues[len(readValues)-1][3] != readValues[len(readValues)-2][3]):
            rate.sleep()

    with open("motorData-"+str(int(time()))+".csv", 'w') as f: 
        write = csv.writer(f) 
        write.writerow(readValuesLabel) 
        write.writerows(readValues)
    
    print("SAVED")

if __name__ == '__main__':
    main()
