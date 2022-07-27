#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped

class control():
    def __init__(self):
        self.distance_sub = rospy.Subscriber('/distance_data', Float64MultiArray, self.distanceCallback)
        self.position_sub = rospy.Subscriber('/position_data', Float64MultiArray, self.positionCallback)
        self.velocity_sub = rospy.Subscriber('/object_speed_data', Float64MultiArray, self.velocityCallback)
        self.ackermann_sub = rospy.Subscriber('/vesc/sns_msg2', AckermannDriveStamped, self.ackermannCallback)
        #self.lane_speed_sub = rospy.Subscriber('')
        #self.lane_steering_sub = rospy.Subscriber('')


        self.distance = np.array([])
        self.position = np.array([])
        self.object_speed = np.array([])
        self.vehicle_speed = 0.0
        self.vehicle_steering = 0.0


    def distanceCallback(self, msg):
        self.distance = np.array(msg.data)
        print('distance', self.distance)

    def positionCallback(self, positionList):
        self.position = np.reshape(np.array(positionList.data), (int((len(positionList.data) / 3)), 3))
        print('position', self.position)
    def velocityCallback(self, msg):
        self.object_speed = np.array(msg.data)
        print('object speed', self.object_speed)

    def ackermannCallback(self, msg):
        self.vehicle_speed = msg.drive.speed
        self.vehicle_steering = msg.drive.steering_angle

    def run(self):
        if self.distance <0.5:
            self.vehicle_speed = 0

if __name__ == "__main__":
    try:
        rospy.init_node("LaneDetection")
        cnt = control()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
