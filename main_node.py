#!/usr/bin/env python
from cv_bridge import CvBridge
import cv2
import rospy
from math import sqrt
from drone import DroneFlight

from cv_bridge import CvBridge
import cv2 as cv 

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
    self.tp = [0, 0, 10, 0]
    self.wp = [[10, 0, 10, 0], [0, 0, 10, 0]]
    self.vp = [[1, 0, 0, 0, 0], [-1, 0, 0, 0, 0]]
    self.takeoff = False
    self.current_wp = 0
    self.current_vp = 0
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
  
  def process(self):
    r = rospy.Rate(3)
    while not rospy.is_shutdown() and self.initialized and not self.completed:
      if not self.d.isArmed():
        self.initialized = False
        continue

      # Input
      lp = self.d.getLocalPosition()
      dist = sqrt((self.wp[self.current_wp][0] - lp.x)**2 + (self.wp[self.current_wp][1] - lp.y)**2 + (self.wp[self.current_wp][2] - lp.z)**2)
      s = "%+.2f %+.2f %+.2f D %.2f" % (lp.x, lp.y, lp.z, dist)
      rospy.loginfo(s)

      # Process
      self.update_wp_reach()
      if self.completed:
        break
      self.image_show()

      # Output

      if not self.takeoff:
        self.d.setLocalPosition(self.tp[0],
                                self.tp[1],
                                self.tp[2],
                                self.tp[3])
      else:
        #self.d.setLocalPosition(self.wp[self.current_wp][0], 
        #                        self.wp[self.current_wp][1], 
        #                        self.wp[self.current_wp][2], 
        #                        self.wp[self.current_wp][3])
      
        self.d.setVelocity(self.vp[self.current_vp][0],
                          self.vp[self.current_vp][1])

      # ImageOpenCV
      bridge = CvBridge()
      
      cv2_img = bridge.imgmsg_to_cv2(self.d.depth_image_sv, 'passthrough')

      cv.imshow('test', cv2_img)
      cv.waitKey(0)
      cv.destroyAllWindows()

      r.sleep()
    
    while not rospy.is_shutdown() and self.initialized:
      self.d.setLocalPosition(0, 0, 2.1)

      lp = self.d.getLocalPosition()
      dist = sqrt(lp.x**2 + lp.y**2 + (2.1 - lp.z)**2)
      if dist < 0.5:
        break

      r.sleep()

    rospy.loginfo("Mode Setting...")
    self.d.setMode("AUTO.LAND")
    while not rospy.is_shutdown() and self.d.getMode() != "AUTO.LAND":
      pass
    rospy.loginfo("Mode [AUTO.LAND] Setted")
  
  def update_wp_reach(self):
    lp = self.d.getLocalPosition()
    
    if not self.takeoff:
      dist = sqrt((self.tp[0] - lp.x)**2 + (self.tp[1] - lp.y)**2 + (self.tp[2] - lp.z)**2)

      if dist < 0.5:
        rospy.loginfo("Take Off")
        self.takeoff = True
    else:
      dist = sqrt((self.wp[self.current_wp][0] - lp.x)**2 + (self.wp[self.current_wp][1] - lp.y)**2)

      if dist < 0.5:
        rospy.loginfo("WayPoint Reached")
        self.current_wp += 1
        self.current_vp += 1
        if len(self.wp) == self.current_wp:
          rospy.loginfo("Mission Complete")
          self.completed = True
  
  def image_show(self):
    img = self.br.imgmsg_to_cv2(self.d.depth_image_sv)
    cv2.imshow('depth', img)
    cv2.waitKey(1)

if __name__ == '__main__':
  rospy.init_node('main', anonymous=True)

  d = DroneControl()
  d.init()
  d.process()
