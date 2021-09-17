import rospy
from math import radians
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from mavros_msgs.msg import State, ExtendedState, HomePosition, AttitudeTarget, Altitude
from sensor_msgs.msg import Image
from mavros_msgs.srv import CommandBool, SetMode

class Drone(object):
  def __init__(self):
    self.home_set = False

    # Publisher Variables
    self.local_position_pv = PoseStamped()
    self.velocity_pv = Twist()
    self.attitude_pv = AttitudeTarget()

    # Subscriber Variables
    self.state_sv = State()
    self.extended_sv = ExtendedState()
    self.home_position_sv = HomePosition()
    self.local_position_sv = PoseStamped()
    self.local_velocity_sv = TwistStamped()
    self.altitude_sv = Altitude()
    self.scene_sv = Image()
    self.depth_image_sv = Image()

    # Publisher Init
    self.LocalPositionPb = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    self.VelocityPb = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
    self.AttitudePb = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

    # Subscriber Init
    rospy.Subscriber('/mavros/state', State, self.stateCb)
    rospy.Subscriber('/mavros/extended_state', ExtendedState, self.extendedCb)
    rospy.Subscriber('/mavros/home_position/home', HomePosition, self.homeCb)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.localPositionCb)
    rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, self.localVelocityCb)
    rospy.Subscriber('/mavros/altitude', Altitude, self.altitudeCb)
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
  
  def pubAttitude(self):
    self.AttitudePb.publish(self.attitude_pv)

  # Subscriber Subscribe(Callback)
  def stateCb(self, msg):
    self.state_sv = msg
  
  def extendedCb(self, msg):
    self.extended_sv = msg
  
  def homeCb(self, msg):
    self.home_set = True
    self.home_position_sv = msg
  
  def localPositionCb(self, msg):
    self.local_position_sv = msg
  
  def localVelocityCb(self, msg):
    self.local_velocity_sv = msg
  
  def altitudeCb(self, msg):
    self.altitude_sv = msg

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
  
  def setVelocity_(self, x, y):
    v = Twist()

    v.linear.x = x
    v.linear.y = y

    self.velocity_pv = v
  
  def setAttitude_(self, r, p, y):
    a = AttitudeTarget()

    a.type_mask = AttitudeTarget.IGNORE_ROLL_RATE \
                | AttitudeTarget.IGNORE_PITCH_RATE \
                | AttitudeTarget.IGNORE_YAW_RATE

    q = quaternion_from_euler(r, p, y)
    a.orientation.x = q[0]
    a.orientation.y = q[1]
    a.orientation.z = q[2]
    a.orientation.w = q[3]

    a.thrust = 0.49
    alt = self.getAltitude()
    v = self.getLocalVelocity()
    if alt < 2.0 and v < 0.05:
      a.thrust += 0.0055
      if alt < 1.9:
        a.thrust += 0.015
        if alt < 1.8:
          a.thrust += 0.1
    elif alt > 2.0 and v > -0.05:
      a.thrust -= 0.006
      if alt > 2.3:
        a.thrust -= 0.006
        if alt > 2.4:
          a.thrust -= 0.1

    self.attitude_pv = a

  # Publisher Publish with Data
  def setLocalPosition(self, x, y, z, yaw=0):
    self.setLocalPosition_(x, y, z, yaw)
    self.pubLocalPosition()
  
  def setVelocity(self, x, y):
    self.setVelocity_(x, y)
    self.pubVelocity()
  
  def setAttitude(self, r, p, y):
    self.setAttitude_(radians(r), radians(p), radians(y))
    self.pubAttitude()

  # Subscriber Data Consumption
  def isFCUConnected(self):
    return self.state_sv.connected
  
  def getMode(self):
    return self.state_sv.mode

  def isArmed(self):
    return self.state_sv.armed
  
  def isGrounded(self):
    return self.extended_sv.landed_state == ExtendedState.LANDED_STATE_ON_GROUND

  def isHomeSet(self):
    return self.home_set
  
  def getLocalPosition(self):
    return self.local_position_sv.pose.position
  
  def getLocalVelocity(self):
    return self.local_velocity_sv.twist.linear.z
  
  def getAltitude(self):
    return self.altitude_sv.local

  def getScene(self):
    return self.scene_sv
  
  def getDepth(self):
    return self.depth_image_sv
  
  # Service Invoke
  def setMode(self, mode):
    self.set_mode(base_mode=0, custom_mode=mode)

  def setArm(self, arm):
    self.arming(arm)