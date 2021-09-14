#!/usr/bin/env python
from cv_bridge import CvBridge
import cv2
import rospy
from math import sqrt
from drone import DroneFlight

import numpy as np

class DroneControl:
  def __init__(self):
    self.d = DroneFlight()
    self.initialized = False
    self.completed = False
    #self.wp =  [[0, 0, 2.1, 0],
    #           [5.5, 0, 2.1, 0], [6.5, 0, 2.1, 0],
    #           [11.5, 2.5, 2.1, 0], [12.5, 2.5, 2.1, 0],
    #           [17.5, 0, 2.1, 0], [18.5, 0, 2.1, 0],
    #           [23.5, 2.5, 2.1, 0], [24.5, 2.5, 2.1, 0],
    #           [29, 2.5, 2.1, -90], [29, -7.5, 2.1, -90], [29, -8.5, 2.1, -90],]
    self.tp = [0, 0, 2.1, 0]
    self.wp = [['v', [5, 0, 2.1, 0], [1, 0]], ['v', [0, 0, 2.1, 0], [-1, 0]], ['p', [5, 0, 2.1, 0]], ['p', [0, 0, 2.1, 0]]]
    self.current_wp = 0
    self.br = CvBridge()

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

      if dist < 0.5:
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

      r.sleep()
  
  def update_wp_reach(self):
    lp = self.d.getLocalPosition()
    
    dist = sqrt((self.wp[self.current_wp][1][0] - lp.x)**2 + (self.wp[self.current_wp][1][1] - lp.y)**2)

    rng = 0.2
    if self.wp[self.current_wp][0] == 'p':
      rng = 0.2
    elif self.wp[self.current_wp][0] == 'v':
      rng = 0.4
    if dist < rng:
      rospy.loginfo("WayPoint Reached")
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

    def sobel_edge(img):
      sobelx = cv2.Sobel(img,cv2.CV_32F,1,0,ksize=1)  # x
      sobelx = cv2.convertScaleAbs(sobelx)
      sobely = cv2.Sobel(img,cv2.CV_32F,0,1,ksize=1)  # y
      sobely = cv2.convertScaleAbs(sobely)
      img_sobel = cv2.addWeighted(sobelx, 1, sobely, 1, 0)
      return img_sobel

    dimg = self.br.imgmsg_to_cv2(self.d.depth_image_sv)
    cv2.imshow('depth', dimg)
    simg = self.br.imgmsg_to_cv2(self.d.scene_sv)
    cv2.imshow('scene', simg)

    simg_HSV = cv2.cvtColor(simg,cv2.COLOR_BGR2HSV)
    ORANGE_MIN = np.array([0, 0, 165],np.uint8)
    ORANGE_MAX = np.array([179, 20, 190],np.uint8)
    frame_threshed = cv2.inRange(simg_HSV, ORANGE_MIN, ORANGE_MAX)

    frame_BGR = cv2.cvtColor(frame_threshed, cv2.COLOR_GRAY2BGR)

    cv2.imshow('edge',sobel_edge(frame_threshed))


    # 모서리 찾기 실패 -> 더 정확한 이미지 검색 필요
    contours, _ = cv2.findContours(frame_threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max = 0
    target =0

    for cont in contours:
      # approx = cv2.approxPolyDP(cont, cv2.arcLength(cont, True)*0.02,True)
      # vtc = len(approx)
      vtc= len(cont)

      if vtc == 8:
        x,y,w,h = cv2.boundingRect(cont)
        area = w*h
        if area > max:
          rospy.loginfo(area)
          max = area
          target = cont

    cv2.drawContours(frame_BGR, [target], 0, (0,255,0), 3)
    cv2.imshow('cont', frame_BGR)

    cv2.waitKey(1)
  
  def land(self):
    r = rospy.Rate(3)
    while not rospy.is_shutdown() and self.initialized:
      self.d.setLocalPosition(0, 0, 2.1)

      lp = self.d.getLocalPosition()
      dist = sqrt(lp.x**2 + lp.y**2 + (2.1 - lp.z)**2)
      if dist < 0.3:
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
