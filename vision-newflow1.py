#!/usr/bin/env python

from scipy import ndimage 
import numpy as np
import cv2
import freenect
import math,rospy
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist 

help_message = '''

Tuslar:
 1 - HSV flow visualization
 2 - glitch

'''
c0= 0.0004
x0=0.07
X=[]
Y=[]
funa =[]
OF =[]
t=[]
time=0
kernel_1=np.array([ [0.0833 , 0.1666 , 0.0833] , [0.1666 ,0.000 ,0.1666] , [0.0833 , 0.1666 , 0.0833] ] )
print kernel_1

flag=100

def get_video(): 
    array,_ = freenect.sync_get_video() 
    return array


def func(opticalw,of1,of2,k):
    a,b,c,d,x=of1,of2,of2-of1,of1+of2,opticalw
    if x<=a:
        #x0=0.05
        c0=0.0001
        return 0.5
    elif ((x>a) and (x<=d/2)):
        #x0=0.05
        #c0=0.00000001
        return (1+2*k*math.pow((x-a)/c,2))
    elif ((x>d/2)and(x<=b)):
        #x0=0.05
        #c0=0.00000005
        return k-2*k*math.pow((x-b)/c,2)
    elif x>b :
        #x0=0.05
        #c0=0.000001
        return k

def turevler(prev,curr):
    Dx = cv2.Sobel(prev,cv2.CV_32F,1,0,ksize=1)
    Dy = cv2.Sobel(prev,cv2.CV_32F,0,1,ksize=1)
    Dt = curr-prev
    return Dx,Dy,Dt
def HSOF(prev,curr,alpha,itr):
    h, w = curr.shape[:2]
    flow=np.zeros((h,w,2),np.float32)
    Dx,Dy,Dt =turevler(prev,curr)
    for i in range(itr):
        uAvg=ndimage.convolve(flow[:,:,0],kernel_1,mode='constant',cval=0.0)
        vAvg=ndimage.convolve(flow[:,:,1],kernel_1,mode='constant',cval=0.0)
        #uAvg=cv2.filter2D(flow[:,:,0],cv2.CV_32F,kernel_1)
        #vAvg=cv2.filter2D(flow[:,:,1],cv2.CV_32F,kernel_1)
        Y=alpha*alpha + np.multiply(Dx,Dx) + np.multiply(Dy,Dy)
        dyv=np.multiply(Dy,vAvg)
        dxu=np.multiply(Dx,uAvg)
        flow[:,:,0]= uAvg - ( Dx*(dxu+ dyv + Dt ) )/Y
        flow[:,:,1]= vAvg - ( Dy*(dxu + dyv + Dt ) )/Y
    return flow
def draw_flow(img, flow, step=16):
    h, w = img.shape[:2]
    y, x = np.mgrid[step/2:h:step, step/2:w:step].reshape(2,-1)
    fx, fy = flow[y,x].T
    lines = np.vstack([x, y, x+fx, y+fy]).T.reshape(-1, 2, 2)
    lines = np.int32(lines + 0.5)
    vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    cv2.polylines(vis, lines, 0, (0, 255, 0))
    for (x1, y1), (x2, y2) in lines:
        cv2.circle(vis, (x1, y1), 1, (0, 255, 0), -1)
    return vis

def draw_hsv(flow):
    h, w = flow.shape[:2]
    fx, fy = flow[:,:,0], flow[:,:,1]
    ang = np.arctan2(fy, fx) + np.pi
    v = np.sqrt(fx*fx+fy*fy)
    hsv = np.zeros((h, w, 3), np.uint8)
    hsv[...,0] = ang*(180/np.pi/2)
    hsv[...,1] = 255
    hsv[...,2] = np.minimum(v*4, 255)
    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return bgr

def warp_flow(img, flow):
    h, w = flow.shape[:2]
    flow = -flow
    flow[:,:,0] += np.arange(w)
    flow[:,:,1] += np.arange(h)[:,np.newaxis]
    res = cv2.remap(img, flow, None, cv2.INTER_LINEAR)
    return res

if __name__ == '__main__':
    x_min_crop = 200 
    x_max_crop = 400 
    y_max_crop = 480 
    y_min_crop = 340 

    xwin_min = 35
    xwin_max = 65
    ywin_min = 20
    ywin_max = 120
    
    pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10) 
    rospy.init_node('DenseflowControl') 
    twist = Twist() 

    import sys
    print help_message
    try: fn = sys.argv[1]
    except: fn = 0

    prev =get_video()
    prev = prev[y_min_crop:y_max_crop,x_min_crop:x_max_crop]
    prevgray = cv2.cvtColor(prev, cv2.COLOR_BGR2GRAY)
    show_hsv = False
    show_glitch = False
    cur_glitch = prev.copy()
    x,y,xo,yo=0.0,0.0,0.0,0.0
    funnew,funold=0.0,0.0
    while 1:
       
        flag-=1
        img = get_video()
        img = img[y_min_crop:y_max_crop,x_min_crop:x_max_crop]
        cv2.imshow('Original',img)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray,(9,9),2)
        flow = 5*HSOF(prevgray,gray,100,1) #
        frame3 = flow[ywin_min:ywin_max,xwin_min:xwin_max]
        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
        mag1,ang1= cv2.cartToPolar(frame3[...,0], frame3[...,1])
        prevgray = gray 
        
        cv2.imshow('flow', draw_flow(gray, flow))
        if show_hsv:
            cv2.imshow('flow HSV', draw_hsv(flow))
        if show_glitch:
            cur_glitch = warp_flow(cur_glitch, flow)
            cv2.imshow('glitch', cur_glitch)
        optical_left,optical_right,optical_diff,optical_sum,optical_net = 0.0,0.0,0.0,0.0,0.0
        for j in range(100):
            for i in range(140):
                optical_left+=mag[i][j] 

        for j in range(100,200):
            for i in range(140):
                optical_right+=mag[i][j]

        opticalw=0.0
        for j in range(30):
            for i in range(100):
                opticalw+=mag1[i][j]
        OF.append(opticalw)
        time+=1
        t.append(time)
        fun=func(opticalw,100,1500,5)
        if(funold>3):
              funnew=0
        else:
              funnew=fun
        funold=fun
        #fun,x0=1,0.05
        funa.append(fun)
        optical_sum=optical_left+optical_right
        optical_diff=optical_left-optical_right
        optical_net=optical_diff/optical_sum
       
        #calculating theta from optical_net 
        print str(optical_left) +"   " +str(optical_right) +"    "+str(optical_diff)+ '\n'
        print str(opticalw) +"   " +str(fun) +"    "+ '\n'
 
        theta =c0*optical_diff*funnew
        
        xn=x0*math.cos(theta*57.32)
        xo+=xn
        X.append(xo)
        yn=x0*math.sin(theta*57.32)
        yo+=yn
        Y.append(yo)
        print theta
        twist.linear.x=x0
        twist.angular.z = theta
        pub.publish(twist)
        
        ch = 0xFF & cv2.waitKey(5)
        if ch == 27:
            break
        if ch == ord('1'):
            show_hsv = not show_hsv
            print 'HSV flow visualization is', ['off', 'on'][show_hsv]
        if ch == ord('2'):
            show_glitch = not show_glitch
            if show_glitch:
                cur_glitch = img.copy()
            print 'glitch is', ['off', 'on'][show_glitch]
    #fig=plt.plot(funa,OF)
    fig0=plt.plot(t,OF)
    plt.savefig('fig0')
    plt.close('fig0')
    fig1=plt.plot(X,Y)
    plt.savefig('Trajectory') 
    cv2.destroyAllWindows()
