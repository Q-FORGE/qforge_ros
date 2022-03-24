#!/usr/bin/env python
# ball launcher for qforge_ros package
# Subscribes to 

import rospy
from std_msgs.msg import String,Bool,Float64,Float32
from nav_msgs.msg import Odometry
from numpy import NaN, array,append,linalg
from math import pi
from scipy.integrate import solve_ivp
from scipy.spatial.transform import Rotation as R

# Fetch node rate parameter
launcher_rate = rospy.get_param('launcher_rate',10)

# mavros uses ENU convention
target_position = array([[12.5,-3,2]])
wall_normal = array([[-1,0,0]])
position = array([[0,-5,5]])
velocity = array([[0,0,0]])

m = 0.3 # [kg] mass
r = 0.1 # [m] radius
Cd = 0 # sphere drag coefficient
rho = 1.225 # [kg/m^3] air density
A = pi*r*r # ball cross-section area
g = -9.81 # [m/s^2]
t_end = 10 # searching period

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
    # Will it hit the wall plane?
    status_pub = rospy.Publisher('Launcher_status', Bool, queue_size=1)
    # If so, what are the errors between the impact point and the target
    lonERR_pub = rospy.Publisher('Launcher_lonERR', Float64, queue_size=1)
    latERR_pub = rospy.Publisher('Launcher_latERR', Float64, queue_size=1)
    # Ball position and velocity for debugging
    ball_pub = rospy.Publisher('Launcher_ball', String, queue_size=1)

    # publish to magnet gain for testing
    magnet_pub = rospy.Publisher('/red/uav_magnet/gain', Float32, queue_size=1,latch=True)

    rospy.init_node('launcher')
    rospy.Rate(launcher_rate)

    while not rospy.is_shutdown():
        rospy.Subscriber("/red/ball/odometry", Odometry, odometry_callback, queue_size=1, buff_size=2**24)

        x0 = append(position,velocity)
        ball = "position = [%.2f,%.2f,%.2f], velocity = [%.2f,%.2f,%.2f]" %(x0[0],x0[1],x0[2],x0[3],x0[4],x0[5])
        ball_pub.publish(ball)

        sol = solve_ivp(xdot, [0,t_end], x0, method='RK45', events=hit_wall)

        if sol.status == 0:
            # IVP solver reaches the end time without triggering hit_wall event
            status = False
            lonERR = NaN
            latERR = NaN
        elif sol.status == 1:
            status = True
            impact_position = sol.y[:3,-1]
            position_error = impact_position-target_position
            # error calculation assumes the wall is vertical
            lonERR = position_error[0,2]
            latERR = linalg.norm(position_error[0,0:2])
            if abs(lonERR) <= 0.5:
                magnet_pub.publish(0.0)
    
        status_pub.publish(status)
        lonERR_pub.publish(lonERR)
        latERR_pub.publish(latERR)

if __name__ == '__main__':
    try:
        launcher()
    except rospy.ROSInterruptException:
        pass

