#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import *
import numpy as np
import random
import math
from time import sleep

laser_msg = LaserScan()
odometry_msg = Odometry()

roll = pitch = yaw = 0.0
kp = 0.5

orientation = 'L'
#is_turning = False

#TARGETs [X,Y]
target_0 = np.array([8.0,  13.0]) #red
target_1 = np.array([17.0,  4.5]) #blue
target_2 = np.array([16.2, 12.0]) #green
target_3 = np.array([ 2.0,  2.0]) #yellow
targets = np.array([target_0,target_1,target_2,target_3])

target = 0
target_x = targets[target][0]
target_y = targets[target][1]

min_distance = 0.5

velocity = Twist()

# INFO
#print(len(laser_msg.intensities)) # 1081
#print(laser_msg.angle_min) # -2.35rad (-135º)
#print(laser_msg.angle_max) #  2.35rad ( 135º)
#print(laser_msg.angle_increment) #0.004
#print(laser_msg.range_min) # 0.0
#print(laser_msg.range_max) # 30.0
#print(len(laser_msg.ranges)) # 1081

def get_rotation():
  global roll, pitch, yaw
  x_o = odometry_msg.pose.pose.orientation.x
  y_o = odometry_msg.pose.pose.orientation.y
  z_o = odometry_msg.pose.pose.orientation.z
  w_o = odometry_msg.pose.pose.orientation.w
  #print("%.2f %.2f %.2f %.2f" % (x_o, y_o, z_o, w_o))
  (roll, pitch, yaw) = euler_from_quaternion ([x_o, y_o, z_o, w_o])

def turn(target_degree):
  get_rotation()
  target_rad = target_degree * math.pi/180.0
  velocity.angular.z = kp * (target_rad-yaw)
  #pub.publish(velocity)

def turn_left(degree):
  get_rotation()
  rad = degree * math.pi/180.0
  velocity.angular.z = kp * (yaw-rad)
  
def turn_90left():
  global orientation
  
  if(orientation == 'L'):
    turn(90)
    new_orientation = 'N'
  elif(orientation == 'O'):
    turn(-90)
    new_orientation = 'S'
  elif(orientation == 'N'):
    turn(180)
    new_orientation = 'O' 
  elif(orientation == 'S'):
    turn(0)
    new_orientation = 'L'
  else:
    pass
  
  if(math.isclose(velocity.angular.z, 0.0, abs_tol=0.0000001)):
    orientation = new_orientation

def turn_right(degree):
  get_rotation()
  rad = degree * math.pi/180.0
  velocity.angular.z = kp * (yaw-rad)
    
def turn_90right():
  global orientation
  
  if(orientation == 'L'):
    turn(-90)
    new_orientation = 'S'
  elif(orientation == 'O'):
    turn(90)
    new_orientation = 'N'
  elif(orientation == 'N'):
    turn(0)
    new_orientation = 'L' 
  elif(orientation == 'S'):
    turn(-180)
    new_orientation = 'O'
  else:
    pass

  if(math.isclose(velocity.angular.z, 0.0, abs_tol=0.0000001)):
    orientation = new_orientation

def odometry_callback(data):
  global odometry_msg
  odometry_msg = data

def laser_callback(data):
  global laser_msg
  laser_msg = data

def get_angle():
  '''
    .target             .target
    |\                 /|       
    | \               /1|       
    |__\.           ./__|       
    |  /robot   robot\  |
    | /               \ |
    |/                 \|
    .target             .target
    
  cos(theta) = C.O / C.A
  cos(theta) = cos( (t_y - y) / (t_x - x) )
  theta =   arccos( (t_y - y) / (t_x - x) )'''
  
  x = odometry_msg.pose.pose.position.x
  y = odometry_msg.pose.pose.position.y
  
  rad = math.atan2( (target_y-y) , (target_x-x) )
  
  degree = rad * 180.0 / math.pi
  rospy.loginfo('degree: %f' % degree)
  
  return(degree)

'''
#Can I see my target? Case yes, follow until it, case no
#What is the shortest path I can go from where I am?

# Supondo que o robô sempre inicia de frente para X positivo
# O robô só anda no eixo X, pra frente ou para trás
'''

def fsm(state):
  global cont
  next_state = state
  if(state == 'start'):
    rospy.loginfo("START")
    if(cont == 10):
      next_state = 'aim'
    cont+=1
  
  elif(state == 'aim'):
    rospy.loginfo("AIM")
    turn(get_angle())
    if(math.isclose(velocity.angular.z, 0.0, abs_tol=0.0000001)):
      next_state = 'move'

  elif(state == 'move'):
    rospy.loginfo("MOVE")
    velocity.linear.x = 0.1
    # Average     
    r_sensor = sum(laser_msg.ranges[180:420]) / 240 #  45º - 105º [60º]
    f_sensor = sum(laser_msg.ranges[420:660]) / 240 # 105º - 165º [60º]
    l_sensor = sum(laser_msg.ranges[660:900]) / 240 # 165º - 225º [60º]
    #print("l:%.2f f:%.2f r: %.2f" % (l_sensor,f_sensor,r_sensor))    

    #TODO
    x = odometry_msg.pose.pose.position.x
    y = odometry_msg.pose.pose.position.y
    
    if(min(laser_msg.ranges[520:560]) > math.dist([x,y], [x_target, y_target]))
      next_state = 'move'
    elif(l_sensor > r_sensor):
      next_state = 'turn_left'
    else:
      next_state = 'turn_right'
    
  elif(state == 'turn_right'):
    rospy.loginfo("TURN_RIGHT")
    turn_right(15)
    if(math.isclose(velocity.angular.z, 0.0, abs_tol=0.0000001)):
      next_state = 'move'
  
  elif(state == 'turn_left'):
    rospy.loginfo("TURN_LEFT")
    turn_left(15)
    if(math.isclose(velocity.angular.z, 0.0, abs_tol=0.0000001)):
      next_state = 'move'
  
  elif(state == 'idle'):
    rospy.loginfo("IDLE")
    next_state = 'idle'
    sleep(2)
  #state = next_state
  return next_state

if __name__ == "__main__": 
  rospy.init_node("stage_controller_node", anonymous=False)  
  rospy.Subscriber("/base_pose_ground_truth", Odometry, odometry_callback)
  #rospy.Subscriber("/odom", Odometry, odometry_callback)
  rospy.Subscriber("/base_scan", LaserScan, laser_callback)
  pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
  
  rate = rospy.Rate(10) #10hz
  
  state = next_state = 'start'
  cont = 0
  
  while not rospy.is_shutdown():
    x = odometry_msg.pose.pose.position.x
    y = odometry_msg.pose.pose.position.y
    
    #Verifica se chegou ao alvo
    distance = math.sqrt((x-target_x)**2 + (y-target_y)**2)
    
    if(distance > min_distance):
      #rospy.loginfo("T: (%.2f, %.2f)" % (target_x, target_y))
      #rospy.loginfo("P: (%.2f, %.2f)" % (x, y))
      #rospy.loginfo("D: %.2f" % distance)
      rospy.loginfo("Wz: %f" % velocity.angular.z)
      rospy.loginfo("yaw: %f" % yaw)
      
      # aim the target
      #turn(get_angle())
      next_state = fsm(state)
      state = next_state
      
    else:
      velocity.linear.x = 0.0
      velocity.angular.z = 0.0
      rospy.loginfo("Alvo alcancado!!")
    
    pub.publish(velocity)

    rate.sleep()
