#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class LightningMcQueen:
    def __init__ (self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_cb)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0

    # right 
    def image_cb(self, msg):

        #image retrival
        image = self.bridge.imgmsg_to_cv2(msg)

        #filter
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 40, 0, 0])
        upper_yellow = numpy.array([ 120, 255, 255])

        #masked image of the line
        mask = cv2.inRange(hsv,  lower_yellow, upper_yellow)
        masked = cv2.bitwise_and(image, image, mask=mask)

        #create a 20 pixel band for computing
        h, w, d = image.shape
        search_top = h/2
        search_bot = 3 * h/4
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        cv2.imshow("band", mask)

        
        M = cv2.moments(mask)
        # self.logcount += 1
        # print("M00 %d %d" % (M['m00'], self.logcount))

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])

            #centroid
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            #PID controller adds turn if line isn t central
            err = cx - w/2
            self.twist.linear.x = 0.2
            self.twist.angular.z = -float(err) / 1000
            self.cmd_vel_pub.publish(self.twist)
        
        cv2.imshow("image", image)
        cv2.waitKey(3)
   
#init node to run program
rospy.init_node('lighting_mcqueen_eye')     
chaChoW = LightningMcQueen()
rospy.spin()