#!/usr/bin/env python

from cmath import pi
from tokenize import Double
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
# from std_msgs.msg import Float32MultiArray
from quadrotor_msgs.msg import FlatTarget
import math

import matplotlib.pyplot as plt
import matplotlib.font_manager as font_manager
import numpy as np


class datashow(object):

    def __init__(self):
        
        # self.rate = rospy.Rate(20) # 10hz
        # rospy.Subscriber('/px4/gazebo/odom', Odometry, self.gazeboTruth_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavrosPose_cb)
        rospy.Subscriber('/reference/flatsetpoint', FlatTarget, self.referencePose_cb)
        # rospy.Subscriber('/reference/yaw', Float32MultiArray, self.referenceYaw_cb)
        self.recMavrosPose_flag = False
        self.recReferencePose_flag = False
        self.referenceYaw_flag = False
        self.mavrosPose = Pose()
        self.referencePose = FlatTarget()
        self.referenceYaw = 0.0

        self.line_sin = []
        self.sin_list = []
        self.POINTS = 300
        self.sin_list.append([0] * self.POINTS)
        self.sin_list.append([0] * self.POINTS)
        self.sin_list.append([0] * self.POINTS)
        self.sin_list.append([0] * self.POINTS)
        self.indx = 0
        
        self.fig, self.ax = plt.subplots(4,1)
        for i in range(0,4):
            # self.ax.set_ylim([-2, 2])
            # self.ax.set_xlim([0, self.POINTS])
            self.ax[i].set_autoscale_on(True)
            # self.ax.set_xticks(range(0, 100, 10))
            # self.ax.set_yticks(range(-2, 3, 1))
            self.ax[i].grid(True)
            line_sin_i, = self.ax[i].plot(range(self.POINTS), self.sin_list[i], label='Sin() output', color='cornflowerblue')
            self.line_sin.append(line_sin_i)


        self.timer = self.fig.canvas.new_timer(interval=50)
        self.timer.add_callback(self.update)
        

    def gazeboTruth_cb(self,msg):
        # 
        self.mavrosPose = msg.pose.pose
        self.recMavrosPose_flag = True
        # print(msg.pose.position.x)

    def mavrosPose_cb(self,msg):
        # 
        self.mavrosPose = msg.pose
        self.recMavrosPose_flag = True
        # print(msg.pose.position.x)

    def referencePose_cb(self,msg):
        self.referencePose = msg
        self.recReferencePose_flag = True

    # def referenceYaw_cb(self,msg):
    #     self.referenceYaw = msg.data[0]
    #     self.referenceYaw_flag = True

    def update(self):
        
        if(not self.recMavrosPose_flag):
            print("no receive mavros pose!")
            return
        if(not self.recReferencePose_flag):
            print("no receive reference pose!")
            return 
    
        err = []
        err.append(self.referencePose.position.x - self.mavrosPose.position.x)
        err.append(self.referencePose.position.y - self.mavrosPose.position.y)
        err.append(self.referencePose.position.z - self.mavrosPose.position.z)
        
        w = self.mavrosPose.orientation.w
        x = self.mavrosPose.orientation.x
        y = self.mavrosPose.orientation.y
        z = self.mavrosPose.orientation.z
        yaw = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
        err_yaw = self.referencePose.yaw - yaw
        if abs(err_yaw) > pi:
            if err_yaw >= 0:
                err_yaw = err_yaw - 2*pi
            else :
                err_yaw = err_yaw + 2*pi

        err.append(err_yaw*180/pi)

        for i in range(0,4):
            
            temp = self.sin_list[i]
            self.sin_list[i] = temp[1:] + [err[i]]

            max = np.abs(np.max(self.sin_list[i]))
            min = np.abs(np.min(self.sin_list[i]))
            self.ax[i].set_ylim([-min-0.1*min , max+0.1*max])

            self.line_sin[i].set_ydata(self.sin_list[i])
            self.ax[i].draw_artist(self.line_sin[i])
            self.ax[i].figure.canvas.draw()
        # RMS    
        err_x = np.array(self.sin_list[0])
        err_y = np.array(self.sin_list[1])
        err_z = np.array(self.sin_list[2])
        # err_position = err_x**2 + err_y**2 + err_z**2
        err_RMS_x = np.sqrt(np.mean(err_x**2))
        err_RMS_y = np.sqrt(np.mean(err_y**2))
        err_RMS_z = np.sqrt(np.mean(err_z**2))
        print("err_RMS :  %f    %f    %f " %(err_RMS_x,err_RMS_y,err_RMS_z))


if __name__ == '__main__':
    rospy.init_node('listener', anonymous=True)
    t = datashow()
    t.timer.start()
    plt.show()

    # t.start()
