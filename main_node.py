#!/usr/bin/env python
import rospy
from math import sqrt
from drone import DroneFlight

class DroneControl:
  def __init__(self):
    self.d = DroneFlight()
    self.initialized = False
    self.completed = False
    self.wp =  [[0, 0, 2.1],
               [5.5, 0, 2.1], [6.5, 0, 2.1],
               [11.5, 2.5, 2.1], [12.5, 2.5, 2.1],
               [17.5, 0, 2.1], [18.5, 0, 2.1],
               [23.5, 2.5, 2.1], [24.5, 2.5, 2.1],
               [29, 2.5, 2.1], [29, -7.5, 2.1], [29, -8.5, 2.1],]
    self.current_wp = 0

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
    while not rospy.is_shutdown() and self.initialized:
      if not self.d.isArmed():
        self.initialized = False
        continue

      # Input
      lp = self.d.getLocalPosition()
      s = "%+.2f %+.2f %+.2f" % (lp.x, lp.y, lp.z)
      rospy.loginfo(s)

      # Process
      self.update_wp_reach()
      if self.completed:
        break

      # Output
      self.d.setLocalPosition(self.wp[self.current_wp][0], self.wp[self.current_wp][1], self.wp[self.current_wp][2], 0)
      #self.d.setVelocity(1)

      r.sleep()
  
  def update_wp_reach(self):
    lp = self.d.getLocalPosition()
    
    dist = sqrt((self.wp[self.current_wp][0] - lp.x)**2 + (self.wp[self.current_wp][1] - lp.y)**2 + (self.wp[self.current_wp][2] - lp.z)**2)

    if dist < 0.2:
      rospy.loginfo("WayPoint Reached")
      self.current_wp += 1
      if len(self.wp) == self.current_wp:
        rospy.loginfo("Mission Complete")
        self.completed = True

if __name__ == '__main__':
  rospy.init_node('main', anonymous=True)

  d = DroneControl()
  d.init()
  d.process()
