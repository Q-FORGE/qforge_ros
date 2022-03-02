#!/usr/bin/env python

# Commander node for qforge_ros package
# Publishes current vehicle state to 'vehicle_state'
# Subscribes to current zone at 'current_zone'
# Subscribes to altitude warning at 'alt_state'
# Subscribes to emergency status at 'emergency_hold'
# Subscribes to ar lock at 'camera/ar_lock'

import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16

# Fetch node rate parameter
commander_rate = rospy.get_param('commander_rate',5)

# Initialize state parameters
state = 'Landed'
emergency_status = False
alt_state = False
ar_lock = False
current_zone = 1


def commander():

    # Initialize node
    rospy.init_node('commander')
    rate = rospy.Rate(commander_rate)

    # Define vehicle state publisher
    state_pub = rospy.Publisher('vehicle_state', String, queue_size = 1)

    while not rospy.is_shutdown():

        # Check if vehicle is in an emergency state
        if not emergency_status:
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
                        state = 'ball_drop'
                else:
                    state = 'error'
            else:
                state = 'move_to_alt'
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









