#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from tf.transformations import euler_from_quaternion

import math

n = 10
x=0
y=0
pose = [0,0,0]

def Curve(x):
    f = 2*math.sin(x) * math.sin(x/2)
    return f

def Waypoints(t):
    x_f =  []
    for i in range(0,n+1):
        x_f.append(2*math.pi*i/n)
    y_f = [Curve(i) for i in x_f]
    return [x_f,y_f]

def Theta(theta):
    if(theta > math.pi):
        theta = theta - 2*math.pi
        return theta
    if(theta < -math.pi):
        theta = 2*math.pi + theta
        return theta
    else:
        return theta

def go_to_goal(x1,y1, velocity_msg, pub):
    print("go_to_goal")
    print("(x1,y1) = ("+ str(x1) + ", " + str(y1) + ")")
    # dis2= 100
    # dis1=0
    distance = 0
    print("Start Theta : " + str(pose[2]))
    print("End Theta : " + str(math.atan2(pose[1]-y1, pose[0] - x1)))
    while True and distance<5:
        x0 = pose[0]
        y0 = pose[1]
        theta = pose[2]
        # print("(x0,y0), Theta = ("+ str(x0) + ", " + str(y0) + ") " + str(theta)) 

        K_linear = 0.5
        distance = abs(math.sqrt((x1-x0)**2 + (y1-y0)**2))
        # dis1=dis2
        # dis2 = distance
        linear_speed = K_linear*distance
        # print("distance : " + str(distance))
        K_angualar = 4.0
        goal_theta = math.atan2(y1-y0, x1 - x0)
        angular_speed = (goal_theta-theta)*K_angualar

        velocity_msg.angular.z = angular_speed
        velocity_msg.linear.x = linear_speed

        pub.publish(velocity_msg)
        # tmp = distance
        if(distance<0.01):
            print("Reaced at " + str(distance))
            print("Goal Theta : " + str(goal_theta))
            print("Theta : " + str(theta))
            break

def control_loop():
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
            # print("(x0,y0) = ("+ str(x0) + ", " + str(y0) + ")")
            # x1 = 2*math.pi/5
            # y1 = Curve(x1)
            # print("(x1,y1) = ("+ str(x1) + ", " + str(y1) + ")")
            # x1=WayPoints[0][2]
            # y1=WayPoints[1][2]
            # print("(x1,y1) = ("+ str(x1) + ", " + str(y1) + ")")

            # print("Theta: " + str(pose[2]))
            # go_to_goal(WayPoints[0][2], WayPoints[1][2], velocity_msg, pub)
            # print(WayPoints)
            #Working Fine UPTO 3 points
            # Diverging in 3to4
            #Working till 9
           
            for i in range(0,n):
                print("\n")
                print("For Point no. " +  str(i+1))
                print("Goal " + str(WayPoints[0][i+1]) + " " + str(WayPoints[1][i+1]))
                go_to_goal(WayPoints[0][i+1], WayPoints[1][i+1], velocity_msg, pub)
                count=1
            print("Final (x0,y0) = ("+ str(pose[0]) + ", " + str(pose[1]) + ")")
            
                    

        # if(count == 0):
        #     for i in range(0,n):
        #         # x1 = pos.pose.pose.position.x
        #         # y1 = pos.pose.pose.position.y
        #         x1=WayPoints[0][i]
        #         y1=WayPoints[1][i]
        #         x2=WayPoints[0][i+1]
        #         y2=WayPoints[1][i+1]

        #         ebot_theta = theta_inital
        #         print("ebot theta : "  + str(ebot_theta))
                
        #         theta_goal = math.atan((y2-y1)/(x2-x1))
        #         print("theta_goal " + str(theta_goal))
        #         e_theta = theta_goal - ebot_theta
        #         e_theta = Theta(e_theta)
        #         print("e_Theta :  " + str(e_theta))
        #         print("                         ")
        #         theta_inital = theta_goal
                
        #         # velocity_msg.angular.z = theta_goal
        #         P = 10
        #         velocity_msg.angular.z = P*e_theta
        #         velocity_msg.linear.x = 0.7
        #         pub.publish(velocity_msg)
        #         print("Controller message pushed at {}".format(rospy.get_time()))
        #         # rate2 = rospy.Rate(100)
        #         rate.sleep()
        #         rate.sleep()
        #         rate.sleep()
        #         rate.sleep()
        #         rate.sleep()
        #         rate.sleep()
        #         rate.sleep()
        #         rate.sleep()
        #     count = 1

        
        # ebot_theta = pos.pose.pose.orientation.z
        # print("ebot theta : "  + str(ebot_theta))
        velocity_msg.linear.x = 0
        velocity_msg.angular.z = 0
        # ebot_theta = .pose.pose.orientation.z
        # print("ebot theta : "  + str(ebot_theta))
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
        'bright':  min(min(msg.ranges[300:420]), range_max)	,
        'fright':  min(min(msg.ranges[300:420]), range_max)	,
        'front':   min(min(msg.ranges[300:420]), range_max)	,
        'fleft':   min(min(msg.ranges[300:420]), range_max)	,
        'bleft':   min(min(msg.ranges[300:420]), range_max)	,
    }



if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass