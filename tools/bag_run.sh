#! /bin/bash
rosbag record -O qforge_test.bag /red/vehicle_state /red/ar_tag_est /hawk2/vrpn_client/estimated_odometry /red/tracker/input_pose /red/pathfinder/trajectory_vis /red/launch/gunner_status /red/launch/error /red/launch/start_pose  __name:=qforge_bagger
