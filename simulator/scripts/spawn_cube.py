#!/usr/bin/env python3

import rospy
import rospkg
import os
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_cube():
    # Initialize ROS node
    rospy.init_node('spawn_cube', anonymous=True)
    
    # Wait for the Gazebo service to become available
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        # Create a proxy for the spawn service
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("hydrus_simulator") 
        file_path = os.path.join(package_path, "models/cube.sdf")
        
        # Load the SDF model from file
        with open(file_path, "r") as f:
            model_xml = f.read()
        
        # Define the initial pose of the cube
        initial_pose = Pose()
        initial_pose.position.x = 0.0
        initial_pose.position.y = 0.0
        initial_pose.position.z = 0.5  # Set above ground to avoid overlap
        
        # Call the spawn service
        resp = spawn_model(model_name="cube",
                           model_xml=model_xml,
                           robot_namespace="/",
                           initial_pose=initial_pose,
                           reference_frame="world")
        
        rospy.loginfo("Cube spawned successfully")
        
    except rospy.ServiceException as e:
        rospy.logerr("Spawn service call failed: %s", e)

if __name__ == '__main__':
    try:
        spawn_cube()
    except rospy.ROSInterruptException:
        pass
