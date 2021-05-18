#!/usr/bin/env python

import rospy
from os import system
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import *
import numpy as np
import random
import math

laser_msg = LaserScan()
odometry_msg = Odometry()
velocity = Twist()

state = next_state = 'start'
roll = pitch = yaw = 0.0
target_angle = 0.0
TURNING = False
cont = 0
kp = 0.9

#TARGETs [X,Y]
target_0 = np.array([8.0,  13.0]) #red
target_1 = np.array([17.0,  4.4]) #blue
target_2 = np.array([16.2, 12.0]) #green
target_3 = np.array([ 2.0,  2.0]) #yellow
targets = np.array([target_0,target_1,target_2,target_3])

target = 0
target_x = targets[target][0]
target_y = targets[target][1]

# Minimum distance to detected the target
min_distance = 0.5

def show_info():
  global target_x, target_y, velocity
  x = odometry_msg.pose.pose.position.x
  y = odometry_msg.pose.pose.position.y
  distance = math.sqrt((x-target_x)**2 + (y-target_y)**2)  
  
  system('clear')
  
  rospy.loginfo(state.upper())
  rospy.loginfo("T: (%.2f, %.2f)" % (target_x, target_y))
  rospy.loginfo("P: (%.2f, %.2f)" % (x, y))
  rospy.loginfo("D: %.2f" % distance)
  rospy.loginfo("robot angle: %f" % (get_rotation() * 180.0/math.pi))
  rospy.loginfo("robot2target_angle: %f" % (get_robot2target_angle() * 180.0/math.pi))
  rospy.loginfo("robot2target_rel_angle: %f" % get_robot2target_rel_angle())
  #rospy.loginfo("Vz: %f " % velocity.angular.z)
  rospy.loginfo("sensor[%d] = %f " % (get_robot2target_sensor_index(), laser_msg.ranges[get_robot2target_sensor_index()]))
  
  rospy.sleep(0.2)

def go_to_start_pos():
  start_point = Odometry()
  
  start_point.pose.pose.position.x = 8.0
  start_point.pose.pose.position.y = 8.0
  start_point.pose.pose.position.z = 0.0
  
  [x,y,z,w] = quaternion_from_euler(0.0, 0.0, 0.0)
  
  start_point.pose.pose.orientation.x = x
  start_point.pose.pose.orientation.x = y
  start_point.pose.pose.orientation.x = z
  start_point.pose.pose.orientation.x = w

  pub_pos.publish(start_point)

def get_rotation():
  global roll, pitch, yaw
  x_o = odometry_msg.pose.pose.orientation.x
  y_o = odometry_msg.pose.pose.orientation.y
  z_o = odometry_msg.pose.pose.orientation.z
  w_o = odometry_msg.pose.pose.orientation.w
  
  (roll, pitch, yaw) = euler_from_quaternion ([x_o, y_o, z_o, w_o])
  
  return yaw

def move_forward():
  velocity.linear.x = 0.5
  velocity.angular.z = 0.0
  pub.publish(velocity)  

def stop():
  velocity.linear.x = 0.0
  velocity.angular.z = 0.0
  pub.publish(velocity)

def get_turn_target_angle(turn_angle):
  robot_angle = get_rotation() * 180.0 / math.pi
  target_angle = robot_angle + turn_angle
  
  if(target_angle < -180.0):
    return target_angle + 360.0
  elif(target_angle > 180.0):
    return target_angle - 360.0
  else:
    return target_angle

def turn(target_degree):
  robot_angle = get_rotation()
  target_rad = target_degree * math.pi/180.0
  velocity.angular.z = kp * (target_rad - robot_angle)
  pub.publish(velocity)

def turn_right():
  global next_state, target_angle, TURNING
  if(not TURNING):
    target_angle = get_turn_target_angle(-30)
    TURNING = True

  turn(target_angle)

  if(math.isclose(velocity.angular.z, 0.0, abs_tol=0.0000001)):
    stop()
    TURNING = False
    next_state = 'move'

def turn_left():
  global next_state, target_angle, TURNING
  if(not TURNING):
    target_angle = get_turn_target_angle(30)
    TURNING = True
    
  turn(target_angle)
  
  if(math.isclose(velocity.angular.z, 0.0, abs_tol=0.0000001)):
    stop()
    TURNING = False
    next_state = 'move'
  
def start():
  global cont, next_state
  #go_to_start_pos()
  if(cont == 10):
    next_state = 'aim'
  cont+=1  

def aim():
  global next_state
  turn(get_robot2target_angle() * 180.0 / math.pi)
  if(math.isclose(velocity.angular.z, 0.0, abs_tol=0.0000001)):
    next_state = 'move'
    
def odometry_callback(data):
  global odometry_msg
  odometry_msg = data

def laser_callback(data):
  global laser_msg
  laser_msg = data

def get_robot2target_angle():  
  global target_x, target_y
  x = odometry_msg.pose.pose.position.x
  y = odometry_msg.pose.pose.position.y
  target_angle = math.atan2((target_y-y),(target_x-x))

  return target_angle

def get_robot2target_rel_angle():
  # relative angle
  robot_angle = get_rotation()
  robot2target_angle = get_robot2target_angle()
  
  rel_angle = (robot2target_angle - robot_angle) * 180.0 / math.pi
  
  if(rel_angle < -230.0):
    rel_angle = rel_angle + 360.0
  elif(rel_angle > 230.0):
    rel_angle = rel_angle - 360.0
  return (rel_angle)

def get_robot2target_sensor_index():
  # robot2target_rel_angle     sensor_index
  #       -180º            ->  OUT_OF_LIMIT
  #       -135º            ->        0
  #       - 90º            ->      180
  #       - 45º            ->      360
  #          0º            ->      540
  #         45º            ->      720
  #         90º            ->      900
  #        135º            ->     1080
  #        180º            ->  OUT_OF_LIMIT
  
  rel_angle = get_robot2target_rel_angle()
  
  # ATTENTION!!:
  # OUT_OF_LIMITs values does not result in a good behavior of the robot 
  # when sensor doesn't "see" the target  
  if(rel_angle <= -135.0):
    return 0
  elif(rel_angle >= 135.0):
    return 1080
  else:
    return 540 + int(4*rel_angle)

def fsm():
  global next_state, state
  
  if(state == 'start'):
    start()
  
  elif(state == 'aim'):
    aim()

  elif(state == 'move'):
    move_forward()
    
    # TODO:
    # Sometimes get stucked
    r_sensor_ave = sum(laser_msg.ranges[180:500]) / 320 #  45º - 125º [80º]
    r_sensor_min = min(laser_msg.ranges[180:500])
    
    f_sensor_ave = sum(laser_msg.ranges[500:580]) / 80 # 125º - 145º [20º]
    f_sensor_min = min(laser_msg.ranges[500:580])
    
    l_sensor_ave = sum(laser_msg.ranges[580:900]) / 320 # 145º - 225º [80º]
    l_sensor_min = min(laser_msg.ranges[580:900])

    #print("l:%.2f f:%.2f r: %.2f" % (l_sensor,f_sensor,r_sensor))    

    x = odometry_msg.pose.pose.position.x
    y = odometry_msg.pose.pose.position.y

    # FIRST: avoid walls
    #TODO:
    # 1º- The way right or left state are choosed when turning can be improved
    # 2º- Find a good angle range to verify walls in the way
    #
    # range: 105º(420) <-> 165º(660) [60º]
    if(min(laser_msg.ranges[420:660]) <= 0.5): #420-660 #360 - 720
      stop()
      rospy.loginfo("AVOID WALL!!!")
      if(l_sensor_min > r_sensor_min):
        next_state = 'turn_left'
      else:
        next_state = 'turn_right'    

    # SECOND: find a way home
    #
    #roomba size size [0.35 0.35 0.25] #[widht depth height]
    # 
    #  _____l_____
    #  \    |    /       # Isosceles Triangle
    # y'\   |h  /y''     l >= 0.4
    #    \  |  /         h = math.dist([x,y], [target_x, target_y])
    #     \ |a/          y'= y''
    #    __\|/__         a = ?   
    #   |       |       
    #   |roomba |l      sen(a) = C.O / hip
    #   |_______|       sen(a) = l/2 / y'' 
    #       l           sen(a) = l/2 / y''         
    #                   2 * y'' * sen(a) = l
    #                   2 * y'' * sen(a) >= 0.4
    #                   y'' * sen(a) >= 0.2                   
    #
    #                   y''**2 = (l/2)**2 + h**2 
    #                   y'' = math.sqrt( (l/2)**2 + h**2)
    #                   y'' = math.sqrt( (0.2)**2 + math.dist**2)
    #                   y'' = math.sqrt(   0.04   + math.dist**2) 
    #
    #                   y'' * sen(a) >= 0.2
    #                   math.sqrt(0.04 + math.dist**2) * sen(a) >= 0.2
    #                   sen(a) >= 0.2 / math.sqrt(0.04 + math.dist**2)
    #                   a >= arcsen(0.2 / math.sqrt(0.04 + math.dist**2))
    #
    #
    elif(abs(get_robot2target_rel_angle()) > 10.0):
      #TODO:
      # 1) turn the indexes range value dynamics according to 
      #    the robot distance to the target, when there is a straight line
      #    between them
      #
      # 2) change INDEX_RANGE or IF condition at line 308 to variate 
      #    accordingly to dist2target value
      #
      dist2target = math.dist([x,y], [target_x, target_y])
      index2target = get_robot2target_sensor_index() #index = 0-1081
      INDEX_RANGE = 360 # range 90º
      if(index2target >= INDEX_RANGE and index2target <= (1080-INDEX_RANGE)):
        begin = index2target - INDEX_RANGE
        end   = index2target + INDEX_RANGE
        move_on = True
        
        for index in range(begin,end):
          angle_from_index = abs(index2target/4 - index/4)
          
          if(angle_from_index < 4.5): # 3.0
            angle_from_index = 4.5
          
          rad_from_index = angle_from_index * math.pi / 180.0
          
          if(laser_msg.ranges[index] < 0.2 / math.sin(rad_from_index)):
            move_on = False

        if(move_on):
          stop()
          next_state = 'aim'

      elif(r_sensor_min > 0.75 and l_sensor_min > 0.75):
        stop()
        if(index2target < 540):
          next_state = 'turn_right'
        else:
          next_state = 'turn_left'

    # THIRD: keep moving
    else:
      next_state = 'move'
    
  elif(state == 'turn_right'):
    turn_right()
  
  elif(state == 'turn_left'):
    turn_left()

  elif(state == 'idle'):
    next_state = 'idle'
  
  state = next_state

if __name__ == "__main__": 
  # Node
  rospy.init_node("stage_controller_node", anonymous=False)  
  # Subscribers
  rospy.Subscriber("/base_pose_ground_truth", Odometry, odometry_callback)
  #rospy.Subscriber("/odom", Odometry, odometry_callback)
  rospy.Subscriber("/base_scan", LaserScan, laser_callback)
  # Publishers
  pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
  pub_pos = rospy.Publisher("/base_pose_ground_truth", Odometry, queue_size=10)
  
  rate = rospy.Rate(10) #10hz
  
  while not rospy.is_shutdown():
    x = odometry_msg.pose.pose.position.x
    y = odometry_msg.pose.pose.position.y
    
    # Distance to target
    distance = math.sqrt((x-target_x)**2 + (y-target_y)**2)
    
    # Wait to sensor starts
    if(laser_msg.ranges):
      # Checks if reached the target
      if(distance > min_distance):
      
        fsm()
        show_info()
        
      else:
        velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        pub.publish(velocity)
        rospy.loginfo("Target reached!!")

    rate.sleep()
