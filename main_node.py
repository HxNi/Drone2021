#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, HomePosition
from sensor_msgs.msg import Image
from mavros_msgs.srv import CommandBool, SetMode

class Drone(object):
  def __init__(self):
    self.home_set = False

    # Publisher Variables
    self.local_position_pv = PoseStamped()

    # Subscriber Variables
    self.state = State()
    self.home_position = HomePosition()
    self.local_position_sv = PoseStamped()
    self.scene = Image()
    self.depth_image = Image()

    # Publisher Init
    self.LocalPositionPb = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # Subscriber Init
    rospy.Subscriber('/mavros/state', State, self.stateCb)
    rospy.Subscriber('/mavros/home_position/home', HomePosition, self.homeCb)

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.localPositionCb)

    rospy.Subscriber('/airsim_node/drone_1/front_center_custom/Scene', Image, self.sceneCb)
    rospy.Subscriber('/simulation/depth_image', Image, self.depthImageCb)

    # Service Client Init
    self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    self.arming = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

  # Publisher Publish
  def pubLocalPosition_(self):
    self.LocalPositionPb.publish(self.local_position_pv)

  # Subscriber Subscribe(Callback)
  def stateCb(self, msg):
    self.state = msg
  
  def homeCb(self, msg):
    self.home_set = True
    self.home_position = msg
  
  def localPositionCb(self, msg):
    self.local_position_sv = msg

  def sceneCb(self, msg):
    self.scene = msg
  
  def depthImageCb(self, msg):
    self.depth_image = msg

class DroneFlight(Drone):
  def __init__(self):
    super(DroneFlight, self).__init__()

  # Publisher Data Generation
  def setLocalPosition(self, x, y, z):
    lp = PoseStamped()
    
    lp.header.stamp = rospy.Time.now()
    lp.pose.position.x = x
    lp.pose.position.y = y
    lp.pose.position.z = z

    self.local_position_pv = lp

  # Publisher Publish with Data
  def pubLocalPosition(self, x, y, z):
    self.setLocalPosition(x, y, z)
    self.pubLocalPosition_()

  # Subscriber Data Consumption
  def isFCUConnected(self):
    return self.state.connected
  
  def getMode(self):
    return self.state.mode

  def isArmed(self):
    return self.state.armed
  
  def isHomeSet(self):
    return self.home_set
  
  def getLocalPosition(self):
    return self.local_position_sv.pose.position
  
  # Service Invoke
  def setMode(self, mode):
    self.set_mode(base_mode=0, custom_mode=mode)

  def setArm(self, arm):
    self.arming(arm)

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
    self.d.pubLocalPosition(0, 0, 0)

    # Set Mode
    rospy.loginfo("Mode Setting...")
    self.d.setMode("OFFBOARD")
    while not rospy.is_shutdown() and self.d.getMode() != "OFFBOARD":
      r.sleep()
      self.d.pubLocalPosition(0, 0, 0)
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
      if not self.d.state.armed:
        self.initialized = False

      # Input
      lp = self.d.getLocalPosition()
      s = "%+.2f %+.2f %+.2f" % (lp.x, lp.y, lp.z)
      rospy.loginfo(s)

      # Process

      # Output
      self.d.pubLocalPosition(6.5, 0, 2.1)

      r.sleep()

if __name__ == '__main__':
  rospy.init_node('main', anonymous=True)

  d = DroneControl()
  d.init()
  d.process()
