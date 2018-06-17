#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist, Vector3, TwistWithCovariance
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import time
import numpy as np
import math
from hark_msgs.msg import HarkSource
from operator import and_
from kobuki_msgs.msg import DigitalInputEvent
from nav_msgs.msg import Odometry

#class for controlling kobuki base
class KobukiBase:
	vel = Twist()
	angle = 0.0
	power = 0.0

#class of main program
class SpeechMove:
	def __init__(self):
		self.azimuth_t = 0.0
		self.diff = 0.0
		self.base = KobukiBase()
		self.sub = rospy.Subscriber('HarkSource',HarkSource,self.EventLoop)
                self.odom_sub = rospy.Subscriber('/odom',Odometry, self.onOdomReceived,queue_size=10)
                self.req_sub = rospy.Subscriber('/speech/localization_req',Bool,self.ReqCB)
                self.pub = rospy.Publisher('/mobile_base/commands/velocity',Twist,queue_size=10)
                self.button = rospy.Subscriber('/mobile_base/events/digital_input',DigitalInputEvent, self.onButtonPressed)
                self.thres = 10.0
                self.digital_in = [True, True, True, True]
                self.odometry = 0.0
                self.predict_angle = 0.0
		self.cond = False
                self.last_cond = self.cond
                self.hz = 100.0
                
                self.spr_flg = False
	
        def ReqCB(self,req):
                self.spr_flg = req.data

        def setThreshold(self, value):
                self.thres = value

        def getThreshold(self):
                return self.thres

        def setFrequency(self, hz):
                self.hz = hz
                
        def getFrequency(self):
                return self.hz

        def logCondition(self, cond):
                self.last_cond = cond

        def setPredictAngle(self, angle):
                self.predict_angle = angle

	def EventLoop(self,message):
                if self.spr_flg == False: #enomoto add
                        return
                self.logCondition(self.cond)
                self.setFrequency(50.0)
                self.setThreshold(29.0)
                self.azimuth_t = self.base.angle

                #calculate condition
                cond = max([message.src[i].power for i in range(len(message.src))])  > self.getThreshold() if len(message.src) > 0 else False
		self.cond = cond

                #calculate angle of speaker
                if self.cond:
                        power_table = np.array([message.src[i].power for i in range(len(message.src))])
                        self.base.power = max(power_table)
			self.base.angle = message.src[np.argmax(power_table)].azimuth

                self.CalcDerivative()
                #calculate prediction angle
                self.CalcPredictAngle()

                #move to the voice	
                self.MoveBase()


        def onButtonPressed(self,message):
                self.digital_in = message.values

        def onOdomReceived(self, message):
                self.odometry = message.twist.twist.angular.z

        def CalcPredictAngle(self):
                #correct prediction angle flag
                angle_cond = (self.last_cond and self.cond) or self.diff > 10

                if angle_cond:
                        #correct angle by sensor info
                        self.setPredictAngle(self.base.angle)
                else:
                        #angle prediction by odometry
                        self.predict_angle -= math.degrees(self.odometry * 1.0/self.getFrequency())

                #range angle from -180 to 180
                if self.predict_angle > 180:
                        self.predict_angle -= 360
                elif self.predict_angle < -180:
                        self.predict_angle += 360


        
        def getPredictAngle(self):
                return self.predict_angle

        def CalcDerivative(self):
                #calculate derivative                                          
                if self.base.angle > 0:
                        self.diff = self.azimuth_t - self.base.angle
                elif self.base.angle < 0:
                        self.diff = self.base.angle - self.azimuth_t
                #rospy.loginfo("derivative of azimuth : %f", self.diff)

        def PIDController(self, mode):
                info = ["true", "prediction"]
                k_p = [2.0, 2.0]
                k_d = [0.6, 0.6]

                if mode == 1:
                        self.base.angle = self.getPredictAngle()

                if self.base.angle >= -30 and 30 >= self.base.angle:
                        rospy.loginfo("arrived %s destination", info[mode])
                        self.base.vel.angular.z = 0.0

                elif self.base.angle > 30:
                        rospy.loginfo("speaker %s angle is positive", info[mode])
                        self.base.vel.angular.z = 0.5 + k_p[mode] * self.base.angle / 180.0 + k_d[mode] * self.diff / 180.0

                elif self.base.angle < -30:
                        rospy.loginfo("speaker %s angle is negative", info[mode])
                        self.base.vel.angular.z = -0.5 + k_p[mode] * self.base.angle /180.0 + k_d[mode] * self.diff / 180.0


        def MoveBase(self):
                if self.digital_in[0] == False:
                        rospy.loginfo("Not moving due to emergency switch")
                        self.base.vel.angular.z = 0.0

                print self.cond
                if self.cond == True:
                        #voice hearing mode
                        rospy.loginfo("Voice hearing")
                        self.PIDController(0)

                elif self.cond == False:
                        #voice prediction mode
                        rospy.loginfo("Voice not hearing")
                        self.PIDController(1)

                rospy.loginfo("speed : %f", self.base.vel.angular.z)
                rospy.loginfo("angle : %f", self.base.angle)
                rospy.loginfo("power : %f", self.base.power)
                self.pub.publish(self.base.vel)
                                
if __name__ == '__main__':
	rospy.init_node('speech_move') 
	speech_move = SpeechMove()	
        rospy.spin()
                
