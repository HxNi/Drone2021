#!/usr/bin/env python
import rospy
from drone import DroneFlight

class DroneControl:
  def __init__(self):
    self.d = DroneFlight()
    self.initialized = False

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

      # Output
      self.d.setLocalPosition(6.5, 0, 2.1)
      self.d.setVelocity(4)

      r.sleep()

if __name__ == '__main__':
  rospy.init_node('main', anonymous=True)

  d = DroneControl()
  d.init()
  d.process()
