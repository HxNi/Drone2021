import rospy
from math import radians
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State, HomePosition
from sensor_msgs.msg import Image
from mavros_msgs.srv import CommandBool, SetMode

class Drone(object):
  def __init__(self):
    self.home_set = False

    # Publisher Variables
    self.local_position_pv = PoseStamped()
    self.velocity_pv = Twist()

    # Subscriber Variables
    self.state_sv = State()
    self.home_position_sv = HomePosition()
    self.local_position_sv = PoseStamped()
    self.scene_sv = Image()
    self.depth_image_sv = Image()

    # Publisher Init
    self.LocalPositionPb = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    self.VelocityPb = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

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
  def pubLocalPosition(self):
    self.LocalPositionPb.publish(self.local_position_pv)
  
  def pubVelocity(self):
    self.VelocityPb.publish(self.velocity_pv)

  # Subscriber Subscribe(Callback)
  def stateCb(self, msg):
    self.state_sv = msg
  
  def homeCb(self, msg):
    self.home_set = True
    self.home_position_sv = msg
  
  def localPositionCb(self, msg):
    self.local_position_sv = msg

  def sceneCb(self, msg):
    self.scene_sv = msg
  
  def depthImageCb(self, msg):
    self.depth_image_sv = msg

class DroneFlight(Drone):
  def __init__(self):
    super(DroneFlight, self).__init__()

  # Publisher Data Generation
  def setLocalPosition_(self, x, y, z, yaw):
    lp = PoseStamped()
    
    lp.header.stamp = rospy.Time.now()
    lp.pose.position.x = x
    lp.pose.position.y = y
    lp.pose.position.z = z

    q = quaternion_from_euler(0, 0, radians(yaw))
    lp.pose.orientation.x = q[0]
    lp.pose.orientation.y = q[1]
    lp.pose.orientation.z = q[2]
    lp.pose.orientation.w = q[3]

    self.local_position_pv = lp
  
  def setVelocity_(self, vel):
    v = Twist()

    v.linear.x = vel
    v.linear.y = vel
    v.linear.z = 0

    self.velocity_pv = v

  # Publisher Publish with Data
  def setLocalPosition(self, x, y, z, yaw = 0):
    self.setLocalPosition_(x, y, z, yaw)
    self.pubLocalPosition()
  
  def setVelocity(self, vel):
    self.setVelocity_(vel)
    self.pubVelocity()

  # Subscriber Data Consumption
  def isFCUConnected(self):
    return self.state_sv.connected
  
  def getMode(self):
    return self.state_sv.mode

  def isArmed(self):
    return self.state_sv.armed
  
  def isHomeSet(self):
    return self.home_set
  
  def getLocalPosition(self):
    return self.local_position_sv.pose.position
  
  # Service Invoke
  def setMode(self, mode):
    self.set_mode(base_mode=0, custom_mode=mode)

  def setArm(self, arm):
    self.arming(arm)