#!/usr/bin/env python3
import pyzed.sl as sl
import cv2
import numpy as np
import rospy
import math
import statistics
import matplotlib.pyplot as plt
import time
from time import sleep
import timeit
from std_msgs.msg import Float64MultiArray
from zed_interfaces.msg import ObjectsStamped


def callback(object):
    for i in range(len(object.objects)):
        print(object.objects[i].position)


if __name__ == "__main__":
    rospy.init_node("object_detection", anonymous=True)
    rospy.Subscriber("/zed2/zed_node/obj_det/objects", ObjectsStamped, callback)
    rospy.spin()

