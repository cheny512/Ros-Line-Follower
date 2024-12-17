#!/usr/bin/env python3
import math
import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Twist, Point
import time

max_angular = 1
max_lidar_dista = 3.5  
min_lidar_dista = 0.1 
spin = 0.2
class LineFollowerReal:
    def __init__(self):
        cv2.namedWindow("window", 1)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/raspicam_node/image/compressed',
                                          CompressedImage,
                                          self.image_cb)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)
        # self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.init_states()
        self.line_detected = False
        self.twist = Twist()
        self.avoid_obstacle = False

        # for wall following 
        self.dista_from_wall = 0.2
        self.front_dista = 1000
        self.right_dist = None
        self.front_right = None
        self.right = None
        self.back_right = None
        self.first_time = True
        self.start_turn_time = None
        self.try_turn = False
        self.mask = None
        self.image = None
        self.w = None

    def init_states(self):
        """Initialize the states of the robot."""
        self.states = {
            'following_line': True,
            'avoiding_obstacle': False,
        }
    


    def initial_turn(self):
        """Turn left for 1 second to clear initial obstacle."""
        print("Initiating left turn for 1 second")
        self.start_turn_time = time.time()  # Record the start time

        # Begin the turn and keep checking the time
        while True:z
            elapsed_time = time.time() - self.start_turn_time  # Calculate elapsed time
            if elapsed_time >= 1.0:  # Exit after 1 second
                break

            # Set the turning twist
            self.twist.angular.z = .4  # Left turn speed
            self.twist.linear.x = 0.03

            self.cmd_vel_pub.publish(self.twist)
        
        # Stop the turn after exiting the loop
        self.twist.angular.z = 0
        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)
        self.first_time = False

    def image_cb(self, msg):
        """Callback to `self.image_sub`."""
        # Use the cv_bridge package to convert ROS sensor_msgs/Image messages 
        # into OpenCV2 images (cf. PRR p.197)
        self.image = self.bridge.compressed_imgmsg_to_cv2(msg,desired_encoding='bgr8')

        height, width = self.image.shape[:2]
        # Set the top half of the image to black
        self.image[:height // 2, :] = 0

        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        lower_red = numpy.array([ 70, 100, 130])
        upper_red = numpy.array([180, 255, 255])
        self.mask = cv2.inRange(hsv, lower_red, upper_red)
        h, self.w, d = self.image.shape
        search_top = round(3*h/4)
        search_bot = search_top + 20
        self.mask[0:search_top, 0:self.w] = 0
        self.mask[search_bot:h, 0:self.w] = 0
        
        
        
    

    def scan_cb(self, msg):
        """Callback function for `self.scan_sub`."""
        front = [r for r in range(360) if r not in range(10, 350)]
        right_angles = range(220, 320)

        front_ranges = [msg.ranges[r] for r in front if min_lidar_dista <= msg.ranges[r] <= max_lidar_dista and msg.ranges[r] != float('inf')]
        right_ranges = [msg.ranges[r] for r in right_angles if min_lidar_dista <= msg.ranges[r] <= max_lidar_dista and msg.ranges[r] != float('inf')]


        front_wall_angles = range(298, 302)
        right_wall_angles = range(262, 278)
        back_wall_angles = range(238, 242)

        front_wall = [msg.ranges[r] for r in front_wall_angles if min_lidar_dista <= msg.ranges[r] <= max_lidar_dista and msg.ranges[r] != float('inf')]
        right_wall = [msg.ranges[r] for r in right_wall_angles if min_lidar_dista <= msg.ranges[r] <= max_lidar_dista and msg.ranges[r] != float('inf')]
        back_wall = [msg.ranges[r] for r in back_wall_angles if min_lidar_dista <= msg.ranges[r] <= max_lidar_dista and msg.ranges[r] != float('inf')]

        self.front_right = min(front_wall) if len(front_wall) > 0 else 5
        self.right = min(right_wall) if len(right_wall) > 0 else 5
        self.back_right = min(back_wall) if len(back_wall) > 0 else 5

        print("front_dist: ", self.front_dista )
        self.right = min(self.right, self.front_right, self.back_right)
        self.front_dista = min(front_ranges) if len(front_ranges) > 0 else None
        self.right_dist = min(right_ranges) if len(right_ranges) > 0 else None
  
    
    def run(self):
        """Run the program."""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            M = cv2.moments(self.mask)
            if M['m00'] and M['m00'] > 0 and self.avoid_obstacle == False:
                self.line_detected = True
                if self.front_dista is not None and self.front_dista < 0.35:
                    self.avoid_obstacle = True
                print("front dista", self.front_dista)
                print("following line red")
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(self.image, (cx, cy), 20, (0,0,255), -1)
                err = (cx - round(self.w/2))/ 300
                self.twist.angular.z = max(min(-float(err), max_angular), -max_angular) 
            
                self.twist.linear.x = 0.1
                self.cmd_vel_pub.publish(self.twist)
                self.first_time = True
            
            elif self.avoid_obstacle == True:
            
                self.line_detected = False

                
                print("wall avoiding")
                print( "M values before", M['m00'] )
                
                # kp = .774
                # kd = 6.4
                if self.first_time == True:
                    self.initial_turn() 
                    M = cv2.moments(self.mask)

                kp = 2
                kd = 6.4

                #Initializing error
                wall_error = self.dista_from_wall - (self.right_dist if self.right_dist else 0)
                prev_error = wall_error
                d_error = 0

                rate = rospy.Rate(10)

                
                wall_error = self.dista_from_wall - (self.right_dist if self.right_dist else 0)
                d_error = wall_error - prev_error if wall_error != prev_error else d_error
                front_error = (self.dista_from_wall) / (self.front_dista + .5)**8  if self.front_dista is not None else 0

                # Adjust angular velocity using PID control and angle adjustment
                turn = (wall_error + front_error ) * kp + d_error * kd
                
                if (self.back_right < self.front_right - .1) or (self.right < self.front_right -.2):
                    turn = -.8

                self.twist.angular.z = turn
                self.twist.linear.x = .1
                # self.twist.angular.z = max(min(turn, max_angular), -max_angular) 
                print ("turn amount",turn, "self.right_dist", self.right_dist)

                self.cmd_vel_pub.publish(self.twist)
                prev_error = wall_error
                print("M values after:", M['m00'])
                if M['m00'] > 0 :
                    self.line_detected = True
                    self.avoid_obstacle = False
                    # rate.sleep()


            # for infinite loop on line
            elif self.line_detected and self.avoid_obstacle == False:
                self.twist.angular.z = 0.5  # Rotate at a higher angular speed for 180-degree turn
                self.twist.linear.x = 0
                self.cmd_vel_pub.publish(self.twist)

                rospy.sleep(2)  

                    # Reset line detection to avoid continuous turning
                self.line_detected = False
                # Line is not detected
                
            # turning to find line, depends on where camera is pointing
            elif self.line_detected == False and self.avoid_obstacle == False:
                # No line detected and no prior detection, do minimal adjustment
                self.twist.angular.z = 0.2
                self.twist.linear.x = 0
                self.cmd_vel_pub.publish(self.twist)
            self.cmd_vel_pub.publish(self.twist)
            rate.sleep()

       
           
if __name__ == '__main__':
    rospy.init_node('line_follower_real')
    LineFollowerReal().run()
