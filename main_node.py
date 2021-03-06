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
    #self.wp = [['a', [5, 0, 0], 1]]
    self.wp =[['v', [1, 0], 3],['p', [7, 0, 2.1,0]], ['p', [9, 1.25, 2.1,0]],
             ['p', [11, 2.5, 2.1,0]], ['p', [13, 2.5, 2.1,0]], ['p', [15, 1.25, 2.1,0]],
            ['p', [17, 0, 2.1,0]], ['p', [19, 0, 2.1,0]], ['p', [21, 1.25, 2.1,0]],
            ['p', [23, 2.5, 2.1,0]], ['p', [25, 2.5, 2.1,0]], ['p', [29, 2.5, 2.1,-90]], 
            ['p', [29, -7, 2.1,-90]], ['p', [29, -9, 2.1,-90]], ['p', [29, -12.9, 2.1,-120]], 
          ['p', [28.25, -13.65, 2.1,-135]], ['p', [26.85, -15.05, 2.1,-135]], ['p', [25.05, -16.85, 2.1,-150]],
            ['p', [22.35, -16.85, 2.1,-180]], ['p', [21.35, -16.85, 2.1,-180]], ['p', [17.85, -16.85, 2.1,-210]], 
            ['p', [15.7, -14.7, 2.1,-225]], ['p', [14.3, -13.3, 2.1,-225]], ['p', [12.25, -11.25, 2.1,-255]], 
          ['p', [12.25, -10.4, 2.1,-270]], ['p', [12.25, -8.4, 2.1,-270]], ['p', [12.25, -6.25, 2.1,-225]],
          ['p', [9.35, -6.25, 2.1,-180]], ['p', [7.35, -6.25, 2.1,-180]], ['p', [5.8, -6.25, 2.1,-135]],
          ['p', [5.8, -10, 2.1,-90]], ['p', [5.8, -12, 2.1,-90]], ['p', [5.8, -12.95, 2.1,-120]], 
          ['p', [5.65, -13.1, 2.1,-135]], ['p', [4.25, -14.5, 2.1,-135]], ['p', [3.95, -14, 2.1,-150]], 
          ['p', [3, -14.8, 2.1,-180]], ['p', [1, -14.8, 2.1,-180]], ['p', [-0.95, -14.8, 2.1,-225]],
          ['p', [-0.95, -8.05, 2.1,-270]], ['p', [-0.95, -6.05, 2.1,-270]], ['p', [0, 0, 2.1,0]]]
    self.detect_rng = 0.2
    self.current_wp = 0
    self.br = CvBridge()
    self.rates_count = 0
    self.rates = 3
    self.post_process = False
    self.post_process_t = 0.5
    self.stuck = False
    self.exdists = self.d.getLocalPosition()
    self.lpCount = 0

  def checkStuck(self):
    lp = self.d.getLocalPosition()
    if lp.x - self.exdists.x < 0.01 and lp.y - self.exdists.y < 0.01:
      self.lpCount+=1
      rospy.loginfo("lpCount : %d" % self.lpCount)
    else:
      self.lpCount = 0
    
    if self.lpCount > 20:
      self.lpCount = 0
      return True
    self.exdists = lp
    return False

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
      
      if dist > 4:
        self.d.setLocalPosition(self.tp[0],
                                self.tp[1],
                                5,
                                self.tp[3])
      else:
        self.d.setLocalPosition(self.tp[0],
                                self.tp[1],
                                self.tp[2],
                                self.tp[3])

      r.sleep()

  def process(self):
    r = rospy.Rate(self.rates)
    while not rospy.is_shutdown() and self.initialized and not self.completed:
      if not self.d.isArmed():
        self.initialized = False
        continue

      # Input
      self.log_position(self.wp[self.current_wp][1])
      try:
        self.image_show()
      except Exception as e:
        rospy.loginfo(str(e))

      # Process
      self.update_wp_reach()
      if self.completed:
        break

      # Output
      if self.stuck is True:
        self.d.setAttitude(0, -4, self.wp[self.current_wp][1][2])
      elif self.wp[self.current_wp][0] == 'p':
        self.d.setLocalPosition(self.wp[self.current_wp][1][0], 
                                self.wp[self.current_wp][1][1], 
                                self.wp[self.current_wp][1][2], 
                                self.wp[self.current_wp][1][3])
      elif self.wp[self.current_wp][0] == 'v':
        if self.post_process is False:
          self.d.setVelocity(self.wp[self.current_wp][1][0],
                             self.wp[self.current_wp][1][1])
        else:
          self.d.setVelocity(0, 0)
      elif self.wp[self.current_wp][0] == 'a':
        if self.post_process is False:
          self.d.setAttitude(self.wp[self.current_wp][1][0], self.wp[self.current_wp][1][1], self.wp[self.current_wp][1][2])
        else:
          self.d.setAttitude(-self.wp[self.current_wp][1][0], -self.wp[self.current_wp][1][1], self.wp[self.current_wp][1][2])

      r.sleep()
  
  def update_wp_reach(self):
    if self.wp[self.current_wp][0] == 'a' or self.wp[self.current_wp][0] == 'v':
      if self.post_process is True:
        if self.rates_count / self.rates >= self.post_process_t:
          self.rates_count = 0
          self.current_wp += 1
          if len(self.wp) == self.current_wp:
            self.completed = True
            rospy.loginfo("Mission Complete")
            return
          rospy.loginfo("Waypoint Setted")
        else:
          self.rates_count += 1
        return
      if self.rates_count / self.rates >= self.wp[self.current_wp][2]:
        self.rates_count = 0
        self.post_process = True
        rospy.loginfo("Objective Reached %s %.2f %.2f" % (self.wp[self.current_wp][0], self.wp[self.current_wp][1][0], self.wp[self.current_wp][1][1]))
      else:
        self.rates_count += 1
      return
    
    lp = self.d.getLocalPosition()
    lo = self.d.getLocalOrientation()
    
    dist = sqrt((self.wp[self.current_wp][1][0] - lp.x)**2 + (self.wp[self.current_wp][1][1] - lp.y)**2)
    orien = abs(self.wp[self.current_wp][1][3] % 360 - lo[2] % 360)
    
    if self.stuck is True:
      if self.rates_count / self.rates >= self.post_process_t:
        self.stuck = False
      else:
        self.rates_count += 1
    elif self.checkStuck():
      self.rates_count = 0
      self.stuck = True 
    elif dist < self.detect_rng and orien < 5:
      rospy.loginfo("WayPoint Reached %.2f %.2f" % (self.wp[self.current_wp][1][0], self.wp[self.current_wp][1][1]))
      self.current_wp += 1
      if len(self.wp) == self.current_wp:
        rospy.loginfo("Mission Complete")
        self.completed = True
  
  def log_position(self, wp):
    lp = self.d.getLocalPosition()
    lo = self.d.getLocalOrientation()
    h = self.d.getAltitude()
    s = ''
    if self.stuck is True:
      s = 'Stuck'
    dist = 0
    if self.wp[self.current_wp][0] == 'p':
      dist = sqrt((wp[0] - lp.x)**2 + (wp[1] - lp.y)**2 + (wp[2] - lp.z)**2)
    s = "%+.2f %+.2f %+.2f D %.2f (%+.2f %+.2f %+.2f) H %.2f %s" % (lp.x, lp.y, lp.z, dist, lo[0], lo[1], lo[2], h, s)
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

      # Depht image making
      ret, dimg_log = cv2.threshold( dimg_log, 30,255, cv2.THRESH_TOZERO)
      ret, dimg_log = cv2.threshold( dimg_log, 75 ,255, cv2.THRESH_TOZERO_INV)
      ret, dimg_e_log = cv2.threshold( dimg_log, 30 ,255, cv2.THRESH_BINARY_INV)

      dimg_e_log_BGR = cv2.cvtColor(dimg_e_log, cv2.COLOR_GRAY2BGR)
      simg_mask = cv2.add(simg, dimg_e_log_BGR)

      cv2.imshow('simg_mask', simg_mask)

      # Scene HSV threshold
      simg_HSV = cv2.cvtColor(simg_mask, cv2.COLOR_BGR2HSV)

      (h, s, v) = cv2.split(simg_HSV)


      ORANGE_MIN = np.array([0, 1, 155],np.uint8)
      ORANGE_MAX = np.array([179, 19, 195],np.uint8)

      frame_threshed = cv2.inRange(simg_HSV, ORANGE_MIN, ORANGE_MAX)

      cv2.imshow('frame_threshed', frame_threshed)
  
      # edge detect
      contours, _ = cv2.findContours(frame_threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
      max = 1600
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

      if type(target) != type(1):
        cv2.drawContours(simg, [target], 0, (255,0,0), 2)

      cv2.imshow('scene + edge detect',simg)
      cv2.waitKey(1)
    except Exception as e:
      rospy.loginfo(e)
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
    while not rospy.is_shutdown() and not self.d.isGrounded():
      pass
    rospy.loginfo("Finished")


if __name__ == '__main__':
  rospy.init_node('main', anonymous=True)

  d = DroneControl()
  d.init()
  d.takeoff()
  d.process()
  d.land()
