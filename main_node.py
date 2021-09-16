#!/usr/bin/env python
from numpy.core.records import array
from cv_bridge import CvBridge
import cv2
import rospy
from math import sqrt
from drone import DroneFlight

import matplotlib.pyplot as plt
import numpy as np

class DroneControl:
  def __init__(self):
    self.d = DroneFlight()
    self.initialized = False
    self.completed = False
    self.tp = [0, 0, 2.1, 0]
    self.wp = [['v', [5, 0, 2.1,0], [1, 0]], ['p', [7, 0, 2.1,0]], ['p', [9, 1.25, 2.1,0]],
             ['p', [11, 2.5, 2.1,0]], ['p', [13, 2.5, 2.1,0]], ['p', [15, 1.25, 2.1,0]],
            ['p', [17, 0, 2.1,0]], ['p', [19, 0, 2.1,0]], ['p', [21, 1.25, 2.1,0]],
            ['p', [23, 2.5, 2.1,0],[1, 0.625]], ['p', [25, 2.5, 2.1,0]], ['p', [29, 2.5, 2.1,-90]], 
            ['p', [29, -7, 2.1,-90]], ['p', [29, -9, 2.1,-90]], ['p', [29, -12.9, 2.1,-120]], 
          ['p', [28.25, -13.65, 2.1,-135]], ['p', [26.85, -15.05, 2.1,-135]], ['p', [25.05, -16.85, 2.1,-150]],
            ['p', [22.35, -16.85, 2.1,-180]], ['p', [21.35, -16.85, 2.1,-180]], ['p', [17.85, -16.85, 2.1,-210]], 
            ['p', [15.7, -14.7, 2.1,-225]], ['p', [14.3, -13.3, 2.1,-225]], ['p', [12.25, -11.25, 2.1,-255]], 
          ['p', [12.25, -10.4, 2.1,-270]], ['p', [12.25, -8.4, 2.1,-270]], ['p', [12.25, -6.25, 2.1,-225]],
          ['p', [9.35, -6.25, 2.1,-180]], ['p', [7.35, -6.25, 2.1,-180]], ['p', [5.8, -6.25, 2.1,-135]],
          ['p', [5.8, -10, 2.1,-90]], ['p', [5.8, -12, 2.1,-90]], ['p', [5.8, -12.95, 2.1,-120]], 
          ['p', [5.65, -13.1, 2.1,-135]], ['a', [4.25, -14.5, 2.1,-135], 3], ['p', [3.95, -14, 2.1,-150]], 
          ['p', [3, -14.8, 2.1,-180]], ['p', [1, -14.8, 2.1,-180]], ['p', [-0.95, -14.8, 2.1,-225]],
          ['p', [-0.95, -8.05, 2.1,-270]], ['a', [-0.95, -6.05, 2.1,-270], 1], ['p', [0, 0, 2.1,0]], ['p', [0, 0, 0, 0]]]
    self.current_wp = 0
    self.br = CvBridge()
    self.stage = 0

  def init(self):
    r = rospy.Rate(4)

    # Check FCU Connection
    rospy.loginfo("FCU Connecting...")
    while not rospy.is_shutdown() and not self.d.isFCUConnected():
      r.sleep()
    rospy.loginfo("FCU Connected")

    # Check Home Setting
    rospy.loginfo("Home Setting...")
    while not rospy.is_shutdown() and not self.d.isHomeSet():
      r.sleep()
    rospy.loginfo("Home Setted")

    # Publish Setpoint
    self.d.setLocalPosition(0, 0, 0)

    # Set Mode
    rospy.loginfo("Mode Setting...")
    self.d.setMode("OFFBOARD")
    while not rospy.is_shutdown() and self.d.getMode() != "OFFBOARD":
      r.sleep()
      self.d.setLocalPosition(0, 0, 0)
      self.d.setMode("OFFBOARD")
    rospy.loginfo("Mode [OFFBOARD] Setted")
    
    # Set Arm
    rospy.loginfo("Arming...")
    self.d.arming(True)
    while not rospy.is_shutdown() and not self.d.isArmed():
      r.sleep()
      self.d.arming(True)
    rospy.loginfo("Armed")
 
    self.initialized = True
    rospy.loginfo("Initialized")
  
  def takeoff(self):
    r = rospy.Rate(3)

    while not rospy.is_shutdown() and self.initialized and not self.completed:
      self.log_position(self.tp)

      lp = self.d.getLocalPosition()
      dist = sqrt((self.tp[0] - lp.x)**2 + (self.tp[1] - lp.y)**2 + (self.tp[2] - lp.z)**2)

      if dist < 0.2:
        rospy.loginfo("Take Off")
        break
      
      self.d.setLocalPosition(self.tp[0],
                              self.tp[1],
                              self.tp[2],
                              self.tp[3])

      r.sleep()

  def process(self):
    r = rospy.Rate(3)
    while not rospy.is_shutdown() and self.initialized and not self.completed:
      if not self.d.isArmed():
        self.initialized = False
        continue

      # Input
      self.log_position(self.wp[self.current_wp][1])
      self.image_show()

      # Process
      self.update_wp_reach()
      if self.completed:
        break

      # Output
      if self.wp[self.current_wp][0] == 'p':
        self.d.setLocalPosition(self.wp[self.current_wp][1][0], 
                                self.wp[self.current_wp][1][1], 
                                self.wp[self.current_wp][1][2], 
                                self.wp[self.current_wp][1][3])
      elif self.wp[self.current_wp][0] == 'v':
        if self.wp[self.current_wp][2] is None:
          rospy.loginfo("No Velocity Waypoint")
        self.d.setVelocity(self.wp[self.current_wp][2][0],
                           self.wp[self.current_wp][2][1])
      if self.wp[self.current_wp][0] == 'a':
        if self.stage / 4 == 0:
          self.d.setLocalPosition(self.wp[self.current_wp][1][0], 
                                  self.wp[self.current_wp][1][1], 
                                  self.wp[self.current_wp][1][2], 
                                  self.wp[self.current_wp][1][3])
        elif self.stage / 4 == 1:
          self.d.setAttitude(self.wp[self.current_wp][2], 0, self.wp[self.current_wp][1][3])
        self.stage += 1
        if self.stage / 4 >= 2:
          self.stage = 0

      r.sleep()
  
  def update_wp_reach(self):
    lp = self.d.getLocalPosition()
    
    dist = sqrt((self.wp[self.current_wp][1][0] - lp.x)**2 + (self.wp[self.current_wp][1][1] - lp.y)**2)

    rng = 0.3
    if self.wp[self.current_wp][0] == 'p':
      rng = 0.3
    elif self.wp[self.current_wp][0] == 'v':
      rng = 0.5
    if dist < rng:
      rospy.loginfo("WayPoint Reached %.2f %.2f" % (self.wp[self.current_wp][1][0], self.wp[self.current_wp][1][1]))
      self.current_wp += 1
      if len(self.wp) == self.current_wp:
        rospy.loginfo("Mission Complete")
        self.completed = True
  
  def log_position(self, wp):
    lp = self.d.getLocalPosition()
    dist = sqrt((wp[0] - lp.x)**2 + (wp[1] - lp.y)**2 + (wp[2] - lp.z)**2)
    s = "%+.2f %+.2f %+.2f D %.2f" % (lp.x, lp.y, lp.z, dist)
    rospy.loginfo(s)
  
  def image_show(self):

    #Logamathic trasform
    def logTransformImage(img):
      c = 255 / np.log1p(np.max(img))
      img_log = c*np.log1p(img)
      # Specify the data type
      img_log = np.array(img_log,dtype=np.uint8)

      return img_log  
    try:
      dimg = self.br.imgmsg_to_cv2(self.d.depth_image_sv)
      simg = self.br.imgmsg_to_cv2(self.d.scene_sv)

      dimg_log = logTransformImage(dimg)

      # Image making
      ret, dimg_log = cv2.threshold( dimg_log, 30,255, cv2.THRESH_TOZERO)
      ret, dimg_log = cv2.threshold( dimg_log, 70 ,255, cv2.THRESH_TOZERO_INV)

      cv2.imshow('dimg_log', dimg_log)

      ret, dimg_e_log = cv2.threshold( dimg_log, 30 ,255, cv2.THRESH_BINARY_INV)

      dimg_e_log_BGR = cv2.cvtColor(dimg_e_log, cv2.COLOR_GRAY2BGR)
      simg_mask = cv2.add(simg, dimg_e_log_BGR)

      #HSV threshold
      simg_HSV = cv2.cvtColor(simg_mask, cv2.COLOR_BGR2HSV)

      (h, s, v) = cv2.split(simg_HSV)


      ORANGE_MIN = np.array([0, 0, 165],np.uint8)
      ORANGE_MAX = np.array([179, 20, 190],np.uint8)

      frame_threshed = cv2.inRange(simg_HSV, ORANGE_MIN, ORANGE_MAX)

      cv2.imshow('frame_threshed', frame_threshed)
  
      # edge detect
      contours, _ = cv2.findContours(frame_threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
      max = 0
      target = 0

      for cont in contours:
        approx = cv2.approxPolyDP(cont, cv2.arcLength(cont, True)*0.02,True)
        vtc = len(approx)

        if vtc == 4:
          x,y,w,h = cv2.boundingRect(cont)
          area = w*h
          if 153600 > area and area > max:
            max = area
            target = cont

      
      cv2.drawContours(simg, [target], 0, (255,0,0), 2)

      cv2.imshow('scene + edge detect',simg)
      cv2.waitKey(1)
    except:
      rospy.loginfo("Error in image_show()")
  
  def land(self):
    r = rospy.Rate(3)
    while not rospy.is_shutdown() and self.initialized:
      self.d.setLocalPosition(0, 0, 2.1)

      lp = self.d.getLocalPosition()
      dist = sqrt(lp.x**2 + lp.y**2 + (2.1 - lp.z)**2)
      if dist < 0.2:
        break

      r.sleep()

    rospy.loginfo("Mode Setting...")
    self.d.setMode("AUTO.LAND")
    while not rospy.is_shutdown() and self.d.getMode() != "AUTO.LAND":
      pass
    rospy.loginfo("Mode [AUTO.LAND] Setted")


if __name__ == '__main__':
  rospy.init_node('main', anonymous=True)

  d = DroneControl()
  d.init()
  d.takeoff()
  d.process()
  d.land()
