#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import math
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from cv_bridge import CvBridge, CvBridgeError
import time

'''
rospy.init_node('LaneDetection')
ack_msg = AckermannDriveStamped()
ack_pub = rospy.Publisher('sns_msg2', AckermannDriveStamped, queue_size=1)
ack_msg.drive.steering_angle = 0.6

while not rospy.is_shutdown():
	ack_pub.publish(ack_msg)
'''


class Camera():
    def __init__(self, ):
        # Open the logitech camera
        #self.cap = cv2.VideoCapture(1)
        self.carmera_sub = rospy.Subscriber('/image_raw', Image, self.process)
        #self.cap.set(3, 1280)
        #self.cap.set(4, 720)

        #self.ret, self.frame = self.cap.read()
        #self.height, self.width, self.layers = self.frame.shape
        self.width=1280
        self.height=720
        self.x1 = 0
        self.x2 = 0
        self.circle = np.zeros((384, 384, 3), np.uint8)

        self.left_line = [int(self.height * (1 / 3)), 720, 551, 320]
        self.right_line = [int(self.height * 2), 720, 820, 320]

        self.red_color = [0, 0, 255]
        self.carCenterCoordinateX = self.width / 2

        self.bridge = CvBridge()
        self.ack_msg = AckermannDriveStamped()
        self.image_msg = Image()
        self.ack_pub = rospy.Publisher('/vesc/sns_msg2', AckermannDriveStamped, queue_size=10)
        self.dect_pub = rospy.Publisher('image_show', Image, queue_size=10)

        self.right_detect = False
        self.left_detect = False
        self.count = 0
        self.prev_left_x2 = 0
        self.prev_right_x2 = 0
        self.prev_center_coordinate = 0

        self.prevLyval = 0
        self.prevRyval = 0
        self.prevLslop = -10000
        self.a = 0.25

    def usb_cam(self, img):
        print("1")
        self.cv_image = self.bridge.imgmsg_to_cv2(img)

    def do_canny(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)  # grayscale
        blur = cv2.GaussianBlur(gray, (5, 5), 0)  # blur
        canny = cv2.Canny(blur, 100, 200)  # canny_edge
        return canny

    def do_segment(self, frame):
        self.height = frame.shape[0]
        self.width = frame.shape[1]
        polygons = np.array([
            [(0, 620), (0, 290), (1280, 290), [1280, 620]]
        ])
        mask = np.zeros_like(frame)
        cv2.fillPoly(mask, polygons, 255)
        segment = cv2.bitwise_and(frame, mask)
        return segment

    def birds_eye_view(self, frame):
        img = frame
        img.shape

        pts1 = np.float32([[303, 313], [38, 468], [1020, 313], [1270, 468]])
        pts2 = np.float32([[50, 50], [50, 900], [900, 50], [900, 900]])

        # cv2.circle(img, (265, 365), 5, (255, 0, 0), -1)
        # cv2.circle(img, (102, 483), 5, (0, 255, 0), -1)
        # cv2.circle(img, (1068, 367), 5, (0, 0, 255), -1)
        # cv2.circle(img, (1253, 512), 5, (0, 0, 0), -1)

        M = cv2.getPerspectiveTransform(pts1, pts2)

        dst = cv2.warpPerspective(img, M, (1100, 1100))
        # print(M)
        return dst

    def calculate_lines(self, frame, lines):
        self.left = []
        self.right = []
        self.left_avg = []
        self.right_avg = []

        # New Code

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line.reshape(4)  # [x1, y1, x2, y2]
                parameters = np.polyfit((x1, x2), (y1, y2), 1)
                slope = parameters[0]
                y_intercept = parameters[1]

                if math.fabs(slope) < 0.4:
                    continue

                if slope < 0:
                    self.left.append((slope, y_intercept))
                else:
                    self.right.append((slope, y_intercept))

        self.right_detect = False
        self.left_detect = False

        if self.left:
            self.left_detect = True
            self.left_avg = np.average(self.left, axis=0)

            self.left_line = self.calculate_coordinates(frame, self.left_avg)

        if self.right:
            self.right_detect = True
            self.right_avg = np.average(self.right, axis=0)
            self.right_line = self.calculate_coordinates(frame, self.right_avg)

        '''
        if self.left:
            self.left_detect = True
            self.left_avg = np.average(self.left, axis=0)
            self.ymax = np.max(self.left,axis=0)[1]
            if self.ymax > 1000:
                self.ymax = self.prevLyval
            self.prevLyval = self.ymax

            if self.prevLslop == -10000:
                tslop = self.left_avg[0]
            else:
                if math.fabs(self.prevLslop - self.left_avg[0]) > 0.05:
                    tslop = self.prevLslop
                else:
                    tslop = self.prevLslop * 0.8 + self.left_avg[0] * 0.2
            self.prevLslop = tslop
            self.left_avg[0] = tslop
            self.left_line = self.calculate_coordinates(frame, self.left_avg, self.ymax)

        if self.right:
            self.right_detect = True
            self.right_avg = np.average(self.right, axis=0)
            self.ymin = np.min(self.right, axis=0)[1]
            self.right_line = self.calculate_coordinates(frame, self.right_avg, self.ymin)
        '''

        return np.array([self.left_line, self.right_line])

    def setNum(self, x_size):
        maxSize = 2147483647
        maxSize2 = -2147483647
        if (x_size > 0):
            num = min(x_size, maxSize)
        else:
            num = max(x_size, maxSize2)

        return num

    def setRange(self):
        self.x1 = self.setNum(self.x1)
        self.x2 = self.setNum(self.x2)

    def setCircle(self, image, left_x, right_x):
        red_color = (0, 0, 255)
        if left_x > 0 and right_x < 1200:
            self.center_point = cv2.circle(image, (int((left_x + right_x) / 2), 500), 10, red_color, 8)
        return self.center_point

    def calculate_coordinates(self, frame, parameters):
        try:
            slope, intercept = parameters
        except TypeError:
            slope, intercept = 0, 0

        y1 = frame.shape[0]
        y2 = int(y1 - 320)

        self.x1 = int((y1 - intercept) / slope)
        self.x2 = int((y2 - intercept) / slope)

        self.setRange()

        return np.array([self.x1, y1, self.x2, y2])

    def visualize_lines(self, frame, lines):
        lines_visualize = np.zeros_like(frame)

        if lines is not None:
            for x1, y1, x2, y2 in lines:
                cv2.line(lines_visualize, (x1, y1), (x2, y2), (0, 255, 0), 20)
        return lines_visualize

    def angle_between_lines(self, laneCenterCoordinate, lines):
        coordinate_subtract = laneCenterCoordinate - self.carCenterCoordinateX

        x1, y1 = (self.carCenterCoordinateX, 400)  # value of line x1, y1
        x2, y2 = (laneCenterCoordinate, lines[0, 3])  # value of line x2, y2

        h = 320
        w = x1 - x2
        tan_theta = math.atan2(w, h) * self.a

        '''
        x1, y1 = (self.carCenterCoordinateX, 400) # value of line x1, y1
        x2, y2 = (laneCenterCoordinate, lines[0,3]) # value of line x2, y2

        center_coordinateY = self.height  # 720

        vector1_x, vector1_y = (self.carCenterCoordinateX - x1, center_coordinateY - y1)
        vector2_x, vector2_y = (self.carCenterCoordinateX - x2, center_coordinateY - y2)

        #print(vector1_x, vector1_y)
        #print(vector2_x, vector2_y)

        dot = vector1_x * vector2_x + vector1_y * vector2_y  # dot product

        vector1_size = math.sqrt(pow(vector1_x, 2) + pow(vector1_y, 2))  # vector1 size

        vector2_size = math.sqrt(pow(vector2_x, 2) + pow(vector2_y, 2))  # vector2 size

        hypo = self.hypo_right_triangle(vector2_size, vector1_size)  # Hypotenuse of right triangle

        theta = math.acos(dot / (vector1_size * vector2_size)) * self.a


        if laneCenterCoordinate >= 640:
            theta *= -1
        '''

        return tan_theta

        # elif coordinate_subtract < 40:
        #     steering =  0.1
        # elif coordinate_subtract > -40:
        #     steering = -0.1
        # elif coordinate_subtract < 60:
        #     steering = 0.2
        # elif coordinate_subtract > -60:
        #     steering = -0.2
        # elif coordinate_subtract < 80:
        #     steering = 0.3
        # elif coordinate_subtract >-80:
        #     steering = -0.3
        # elif coordinate_subtract < 100:
        #     steering = 0.4
        # elif coordinate_subtract > -100:
        #     steering = -0.4
        # elif coordinate_subtract < 120:
        #     steering = 0.5
        # elif coordinate_subtract > -120:
        #     steering = -0.5

    def hypo_right_triangle(self, width, height):
        hypo = math.sqrt(pow(width, 2) + pow(height, 2))
        return hypo

    def circle_curvature(self, hypo, theta):
        curvature = math.sin(theta) / hypo
        # print(curvature)
        return curvature

    def image_publish(self, image):
        cv_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.dect_pub.publish(cv_image)

    def recording_video(self, video):
        size = (self.width, self.height)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter('output_test1.mp4', cv2.VideoWriter_fourcc('a', 'v', 'c', '1'), 20, size)
        if not out.isOpened():
            print("error")
            cap.release()
            sys.exit()
        out.write(video)
        if rospy.is_shutdown():
            out.release()

    def process(self, img):
        cv_image = self.bridge.imgmsg_to_cv2(img)
        # start3 = time.time()
        # start1 = time.time()
        canny = self.do_canny(cv_image)
        # print("canny_time :", time.time() - start1)

        ROI_img = self.do_segment(canny)
        # start2 = time.time()
        hough = cv2.HoughLinesP(ROI_img, 6, np.pi / 60, 160, np.array([]), minLineLength=10, maxLineGap=20)
        # print("Hough :", time.time() - start2)
        # start1 = time.time()
        lines = self.calculate_lines(cv_image, hough)
        # print("calculate_lines :", time.time() - start1)
        # center_circle=self.setCircle((self.frame, lines[0][2], lines[1][2]))

        # length = abs(lines[0][2] - lines[1][2])
        # print(length)

        # TODO get x coord value from each lines center
        laneCenterCoordinates = int((lines[0][2] + lines[1][2]) / 2)

        if len(self.left_avg) != 0 and len(self.right_avg) != 0:
            self.prev_left_x2 = lines[0][2]
            self.prev_right_x2 = lines[1][2]
            self.prev_center_coordinate = laneCenterCoordinates
            # print("normal")

        if len(self.right_avg) == 0:
            # print("only left")
            laneCenterCoordinates = self.prev_center_coordinate + (lines[0][2] - self.prev_left_x2)
            # print("prev_lanecenter", laneCenterCoordinates)
            self.prev_left_x2 = lines[0][2]
            self.prev_center_coordinate = laneCenterCoordinates
            # print("prev_left_x2b", self.prev_left_x2)

        elif len(self.left_avg) == 0:
            # print("only right")
            laneCenterCoordinates = self.prev_center_coordinate + (lines[1][2] - self.prev_right_x2)
            self.prev_right_x2 = lines[1][2]
            self.prev_center_coordinate = laneCenterCoordinates

        center_circle = cv2.circle(cv_image, (laneCenterCoordinates, 400), 10, self.red_color, 8)
        lines_visualize = self.visualize_lines(center_circle, lines)

        steering = self.angle_between_lines(laneCenterCoordinates, lines)
        self.ack_msg.drive.steering_angle = steering
        self.ack_pub.publish(self.ack_msg)
        # print("all :", time.time() - start3)

        output = cv2.addWeighted(cv_image, 0.9, lines_visualize, 1, 1)
        # recording_video(output)


        if output is not None:

            if cv2.waitKey(9) & 0xFF == ord('s'):
                cv2.waitKey(0)

            cv2.imshow("minLineLength=1", output)

        self.image_publish(output)


if __name__ == "__main__":
    try:
        rospy.init_node("LaneDetection")
        camera = Camera()
        cv2.destroyAllWindows()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

