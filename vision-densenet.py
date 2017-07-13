#!/usr/bin/env python
import freenect
import cv2
import numpy as np
import math
import rospy 
from geometry_msgs.msg import Twist 

def get_video(): 
    array,_ = freenect.sync_get_video() 
    return array

theta= 0.0
c=1
kernel = np.ones((3,3),np.uint8) 
 
w,h = 640,480
Mat = [[0 for x in range(w)] for y in range(h)] 
 
if __name__=='__main__':	
    x_min_crop = 150 
    x_max_crop = 450 
    y_max_crop = 480 
    y_min_crop = 200 
    
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10) 
    rospy.init_node('DenseflowControl') 
    twist = Twist() 


    frame1 = get_video()
    frame1 = frame1[y_min_crop:y_max_crop,x_min_crop:x_max_crop]
    prvs = cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
    hsv = np.zeros_like(frame1)
    hsv[...,1] = 255
    while(1):
        frame2 = get_video()
        frame2 = frame2[y_min_crop:y_max_crop,x_min_crop:x_max_crop]
        cv2.imshow('Original',frame2)
	next = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)
        flow = cv2.calcOpticalFlowFarneback(prvs,next, None, 0.5, 3, 5, 3, 5, 1.2, 0)
        #flow = cv2.erode(flow,kernel,iterations = 5) 
        #flow = cv2.dilate(flow,kernel,iterations = 1) 
        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
        hsv[...,0] = ang*180/np.pi/2
        hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
        bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
        cv2.imshow('frame2',bgr)
        
        optical_left,optical_right,optical_diff,optical_sum,optical_net = 0.0,0.0,0.0,0.0,0.0
        for j in range(150):
            for i in range(280):
                optical_left+=mag[i][j] 

        for j in range(150,300):
            for i in range(280):
                optical_right+=mag[i][j]
        
        optical_sum=optical_left+optical_right
        optical_diff=optical_left-optical_right
        optical_net=optical_diff/optical_sum
        
        #calculating theta from optical_diff 
        print str(optical_left) +"   " +str(optical_right) +"    "+str(optical_net)+ '\n'
 
        theta =c*optical_net
        print theta
        twist.linear.x=0.14
        twist.angular.z = theta
        pub.publish(twist)
        

        k = cv2.waitKey(30) & 0xff
        if k == 27:
            break
        elif k == ord('s'):
            cv2.imwrite('opticalfb.png',frame2)
            cv2.imwrite('opticalhsv.png',bgr)
        prvs = next
    cv2.destroyAllWindows()
