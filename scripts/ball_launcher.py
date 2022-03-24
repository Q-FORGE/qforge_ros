#!/usr/bin/env python
# ball launcher for qforge_ros package
# Subscribes to 

import rospy
from std_msgs.msg import String
#from std_msgs.msg import Bool
#from std_msgs.msg import Int16
from nav_msgs.msg import Odometry

from numpy import array,append,linalg
from math import pi
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation as R

# Fetch node rate parameter
launcher_rate = rospy.get_param('launcher_rate',10)

# mavros uses ENU convention
target_position = array([[0,0,0]])
wall_normal = array([[0,-1,0]])
position = array([[0,-5,5]])
velocity = array([[0,0,0]])

m = 0.3 # [kg] mass
r = 0.1 # [m] radius
Cd = 0 # sphere drag coefficient
rho = 1.225 # [kg/m^3] air density
A = pi*r*r # ball cross-section area
g = -9.81 # [m/s^2]
t_end = 20 # searching period

# initial ball status
def xdot(t,x):
    return append(x[3:6],[0,0,g])-(linalg.norm(x[3:6])*Cd*rho*A/2/m)*append([0,0,0],x[3:6])

# event function, outputs 0 when x is on the wall plane
def hit_wall(t,x):
    return (x[0:3]-target_position)@wall_normal.T
hit_wall.terminal = True
hit_wall.direction = -1

def odometry_callback(msg):
    global position,velocity
    position = array([[msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z]])
    r = R.from_quat([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    velocity = r.apply([[msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z]])

def launcher():
    launcher_pub = rospy.Publisher('launcher_status', String)
    ball_pub = rospy.Publisher('ball_status', String)
    rospy.init_node('launcher')
    rate = rospy.Rate(launcher_rate)

    while not rospy.is_shutdown():

        #/red/ball/position geometry_msgs/PointStamped
        #/red/ball/velocity_relative geometry_msgs/TwistStamped
        #/red/velocity_relative geometry_msgs/TwistStamped
        rospy.Subscriber("/red/ball/odometry", Odometry, odometry_callback)

        x0 = append(position,velocity)
        ball_status = "position = [%.2f,%2f,%2f], velocity = [%.2f,%2f,%2f]" %x0[0],x0[1],x0[2],x0[3],x0[4],x0[5]
        ball_pub.publish(ball_status)

        sol = solve_ivp(xdot, [0,t_end], x0, method='RK45', events=hit_wall)

        if sol.status == 0:
            launcher_status = "Will not hit wall in %.2f " %t_end
            launcher_pub.publish(launcher_status)
        elif sol.status == 1:
            launcher_status = "Hit wall in %.2f s" %sol.t_events[0]
            launcher_pub.publish(launcher_status)
            impact_position = sol.y[:3,-1]
            # error calculation assumes the wall is vertical
            position_error = impact_position-target_position
            lat_error = linalg.norm(position_error[0,0:2])
            lon_error = position_error[0,2]

if __name__ == '__main__':
    try:
        launcher()
    except rospy.ROSInterruptException:
        pass

