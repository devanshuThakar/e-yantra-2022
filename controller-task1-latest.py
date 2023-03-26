#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

import math

n = 6
x=0
y=0
pose = [0,0,0]
srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
state = 0
p1x=0
p1y=0
regions = None

def Curve(x):
    f = 2*math.sin(x) * math.sin(x/2)
    return f

def Waypoints(t):
    x_f =  []
    for i in range(0,n+1):
        x_f.append(2*math.pi*i/n)
    y_f = [Curve(i) for i in x_f]
    return [x_f,y_f]

def Range_angle(theta):
    if(math.fabs(theta) > math.pi):
        theta = theta - (2* math.pi * theta)/(math.fabs(theta))
    return theta

def go_to_goal(x1,y1, velocity_msg, pub):
    # print("go_to_goal")
    # print("(x1,y1) = ("+ str(x1) + ", " + str(y1) + ")")
    distance = 0
    # print("Start Theta : " + str(pose[2]))
    # print("End Theta : " + str(math.atan2(pose[1]-y1, pose[0] - x1)))
    while True and distance<5:
        x0 = pose[0]
        y0 = pose[1]
        theta = pose[2]

        K_linear = 0.5
        distance = abs(math.sqrt((x1-x0)**2 + (y1-y0)**2))
        
        linear_speed = K_linear*distance
        K_angualar = 4.0

        goal_theta = math.atan2(y1-y0, x1 - x0)
        angular_speed = (goal_theta-theta)*K_angualar

        velocity_msg.angular.z = angular_speed
        velocity_msg.linear.x = linear_speed

        pub.publish(velocity_msg)
        if(distance<0.01):
            print("Reaced at " + str(distance))
            print("Goal Theta : " + str(goal_theta))
            print("Theta : " + str(theta))
            break

def distance_to_line(x0, y0):
    # p0 is the current position
    # p1 and p2 points define the line
    # p1 = initial_position_
    # p2 = desired_position_
    # here goes the equation
    up_eq = math.fabs((0 - p1y) * x0 - (12.5 - p1x) * y0 + (12.5 * p1y) - (0 * p1x))
    lo_eq = math.sqrt(pow(0 - p1y, 2) + pow(12.5 - p1x, 2))
    distance = up_eq / lo_eq

    return distance

# def change_state(state):
#     global state_, state_desc_
#     global srv_client_wall_follower_, srv_client_go_to_point_
#     state_ = state
#     log = "state changed: %s" % state_desc_[state]
#     rospy.loginfo(log)
#     if state_ == 0:
#         resp = srv_client_go_to_point_(True)
#         resp = srv_client_wall_follower_(False)
#     if state_ == 1:
#         resp = srv_client_go_to_point_(False)
#         resp = srv_client_wall_follower_(True)

def control_loop():
    global state

    rospy.init_node('ebot_controller')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0  
    pub.publish(velocity_msg)
    count = 0


    while not rospy.is_shutdown():
        if(count==0):
            WayPoints = Waypoints(1)
            for i in range(0,n):
                # print("\n")
                # print("For Point no. " +  str(i+1))
                # print("Goal " + str(WayPoints[0][i+1]) + " " + str(WayPoints[1][i+1]))
                go_to_goal(WayPoints[0][i+1], WayPoints[1][i+1], velocity_msg, pub)
                count=1
            print("Final (x0,y0) = ("+ str(pose[0]) + ", " + str(pose[1]) + ")")
        
        p1x = pose[0]
        p1y = pose[1]

        desired_pos_y = 0
        desired_pos_x = 12.5

        go_to_goal(2*math.pi + 1,2.5, velocity_msg, pub)
        go_to_goal(2*math.pi + 3,2.5, velocity_msg, pub)
        go_to_goal(12.5,0, velocity_msg, pub)
        print("Final (x0,y0) = ("+ str(pose[0]) + ", " + str(pose[1]) + ")")
        #Bug Algortihm
        # NotReach = True
        # velocity_msg.linear.x = 0.5
        # velocity_msg.angular.z = 0
        # pub.publish(velocity_msg)
        # while(NotReach):
        #     velocity_msg.angular.z = 0
        #     pub.publish(velocity_msg)
        #     # print(regions)

        #     if regions == None and state==0:
        #         continue

        #     if  state == 0:
        #         if(regions['front'] > 0.15 and regions['front']<0.5):
        #             velocity_msg.linear.x = velocity_msg.linear.x/10
        #             velocity_msg.angular.z = 1
        #             pub.publish(velocity_msg)
        #             state=1
            
        #     elif state == 1:
        #         desired_theta = math.atan2(desired_pos_y-pose[1], desired_pos_x-pose[1])
        #         err_theta = Range_angle(desired_theta - pose[2])
        #         velocity_msg.angular.z = 0.1

        #         print("State 1 start")
        #         pub.publish(velocity_msg)
        #         if(regions['front'] > 1.5 and regions['fright']>1 and regions['fleft']>1):
        #             state=0
        #             print("GO TO Goal Start")
        #             go_to_goal(0,12.5, velocity_msg, pub)
        #             NotReach = False
        

        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
    	pub.publish(velocity_msg)
    	print("Controller message pushed at {}".format(rospy.get_time()))
    	rate.sleep()


def odom_callback(data):
    global pose
    x  = data.pose.pose.orientation.x
    y  = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x,y,z,w])[2]]

def laser_callback(msg):
    global regions
    range_max =720
    regions = {
        # 'bright':  min(min(msg.ranges[300:420]), range_max)	,
        # 'fright':  min(min(msg.ranges[300:420]), range_max)	,
        # 'front':   min(min(msg.ranges[300:420]), range_max)	,
        # 'fleft':   min(min(msg.ranges[300:420]), range_max)	,
        # 'bleft':   min(min(msg.ranges[300:420]), range_max)	,
        'bright':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'bleft':   min(min(msg.ranges[576:719]), 10),
    }

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass