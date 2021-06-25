#!/usr/bin/env python

import rospy
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged
from darknet_ros_msgs.msg import ObjectCount
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class detectObj:

    def __init__(self):
        rospy.init_node('found_obj_gps')#, anonymous=True)
        
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/PositionChanged', \
                         Ardrone3PilotingStatePositionChanged, self.get_gps_cb)
        rospy.Subscriber('/darknet_ros/found_object', ObjectCount, self.found_obj_cb) 
        rospy.Subscriber('/darknet_ros/detection_image', Image, self.image_cb)
        
        self.gps_info = Ardrone3PilotingStatePositionChanged()
        self.obj_cnt  = ObjectCount()
        self.bridge   = CvBridge()
        self.img_msg  = Image()
        

    def get_gps_cb(self, msg):
        self.gps_info = msg

    def found_obj_cb(self, msg):
        self.obj_cnt  = msg
        print "(count: %s)" %(self.obj_cnt.count)
        
        if self.obj_cnt.count > 0:
            print "(%s, %s)" %(self.gps_info.latitude, self.gps_info.longitude)
            
            try:
                cv2_img = self.bridge.imgmsg_to_cv2(self.img_msg, "bgr8")
            except CvBridgeError, e:
                print(e)
            else:
                path = '/home/dk/'
                fname = path + str(self.gps_info.latitude) + '_' +str(self.gps_info.longitude) + '.jpg'
                print "image saved as %s!" %(fname)
                cv2.imwrite(fname, cv2_img)
            
    
    def image_cb(self, msg):
        print("Received an image!")
        self.img_msg  = msg



if __name__ == '__main__':
    
    detectObj()
    rospy.spin()
