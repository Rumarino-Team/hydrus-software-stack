#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState


def spawn_cube():
    # Initialize ROS node
    rospy.init_node('spawn_cube_node')
    
    # Wait for the spawn service
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    
    try:
        # Create a proxy for the spawn service
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        
        # Read the cube SDF file
        with open("cube.sdf", "r") as f:
            cube_sdf = f.read()
        
        # Define the initial pose of the cube
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 1.0
        
        # Call the spawn service
        spawn_model(model_name="cube", model_xml=cube_sdf, robot_namespace="", initial_pose=pose, reference_frame="world")
        rospy.loginfo("Cube spawned successfully!")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def move_cube():
    # Wait for the set_model_state service
    rospy.wait_for_service('/gazebo/set_model_state')
    
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # Define the new state for the cube
        state_msg = ModelState()
        state_msg.model_name = 'cube'
        state_msg.pose.position.x = 0.0
        state_msg.pose.position.y = 0.0
        state_msg.pose.position.z = 1.0
        state_msg.twist.linear.x = 20.0  # Move along the x-axis
        state_msg.twist.linear.y = 0.0
        state_msg.twist.linear.z = 0.0
        state_msg.reference_frame = 'world'
        
        # Set the cube state
        set_state(state_msg)
        rospy.loginfo("Cube moved successfully!")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


if __name__ == '__main__':
    spawn_cube()
    rospy.sleep(2)  # Wait for Gazebo to stabilize
    move_cube()
