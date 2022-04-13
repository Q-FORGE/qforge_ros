#!/usr/bin/env python

# Commander node for qforge_ros package
# Publishes String current vehicle state to 'vehicle_state'
# Subscribes to Int16 current zone at 'current_zone'
# Subscribes to Bool altitude warning at 'alt_state'
# Subscribes to Bool  emergency status at 'emergency_hold'
# Subscribes to Bool ar lock at 'camera/ar_lock'
# Subscribes to Bool ball_dropped at 'launch/ball_dropped'
# Subscribes to Bool drop_primed at 'launch/drop_primed'

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16

# Fetch node rate parameter
commander_rate = rospy.get_param('commander_rate',5)

# Initialize state parameters
state = 'takeoff'
emergency_status = False
alt_state = False
ar_lock = True
ball_dropped = False
drop_primed = False
drop_progress = False
current_zone = 1

def alt_callback(msg):
    # Update 'alt_state' when topic is published
    global alt_state
    alt_state = msg.data

def drop_callback(msg):
    # Update 'ball_dropped' when topic is published
    global ball_dropped
    ball_dropped = msg.data

def ready_callback(msg):
    # Update 'drop_primed' when topic is published
    global drop_primed
    drop_primed = msg.data

def commander():

    global state

    # Initialize node
    rospy.init_node('commander')
    rate = rospy.Rate(commander_rate)

    # Define vehicle state publisher
    state_pub = rospy.Publisher('vehicle_state', String, queue_size = 1)

    # Define subscribers for state booleans
    alt_sub = rospy.Subscriber('alt_state', Bool, alt_callback)
    drop_sub = rospy.Subscriber('launch/ball_dropped', Bool, drop_callback)
    ready_sub = rospy.Subscriber('launch/drop_primed', Bool, ready_callback)

    while not rospy.is_shutdown():

        # Check if vehicle is in an emergency state
        if not emergency_status:
            # Check if vehicle has completed takeoff
            if not state == 'takeoff':
                # Check if vehicle is within altitude bounds
                if alt_state:
                    # Decide on state based on current zone
                    if current_zone == 1:
                        state = 'trans_12'
                    elif current_zone == 2:
                        state = 'trans_23'
                    elif current_zone == 3:
                        # Switch to Task 3 if tag found, Task 2 if not.
                        if not ar_lock:
                            state = 'ar_search'
                        else:
                            # Check if ball has already been dropped
                            if not ball_dropped:
                                # Check if drop start position was achieved
                                if drop_primed:
                                    state = 'ball_drop'
                                    drop_progress = True
                                # Check if ball drop is currently in progress
                                elif drop_progress:
                                    state = 'ball_drop'
                                else:
                                    state = 'trans_to_drop'
                            else:
                                state = 'mission_complete'
                                drop_progress = False
                    else:
                        state = 'error'
                else:
                    state = 'move_to_alt'
            # Mark takeoff as completed if vehicle moves to altitude
            elif alt_state:
                state = 'takeoff_complete'
        else:
            state = 'error'

        state_pub.publish(state)
        rospy.loginfo(state)
        rate.sleep()

if __name__ == '__main__':
    try:
        commander()
    except rospy.ROSInterruptException:
        pass









