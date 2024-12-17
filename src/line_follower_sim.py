#!/usr/bin/env python3
#VIDEO FOR BOTH SIM AND REAL: https://drive.google.com/file/d/15sydWVxFG1Ig6gDdVnQMinNQ86Lbu5n1/view?usp=sharing
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
max_angular = 0.5
class LineFollowerSim:
    def __init__(self):
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',
                                          Image,
                                          self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        # self.cx = None
        # self.err = None
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 10, 10, 10])
        upper_yellow = numpy.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        h, w, d = image.shape
        search_top = round(3*h/4)
        search_bot = search_top + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            err = (cx - round(w/2))/ 1000
            self.twist.angular.z = max(min(-float(err), max_angular), -max_angular) 
        
            self.twist.linear.x = 0.2
            self.cmd_vel_pub.publish(self.twist)
        # cv2.imshow("window", image)
        # cv2.waitKey(3)
    
   


    def run(self):
        """Run the Program."""
        rate = rospy.Rate(10)
    
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('line_follower_sim')
    LineFollowerSim().run()