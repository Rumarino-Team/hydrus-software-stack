#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu
from actionlib import SimpleActionClient
from autonomy.msg import NavigateToWaypointAction

class PrequalificationTester:
    def __init__(self):
        rospy.init_node('prequalification_tester', anonymous=True)

        # Flags and storage
        self.depth_received = False
        self.imu_received = False
        self.thruster_values = {}

        # Subscribers 
        rospy.Subscriber('/hydrus/depth', Int16, self.depth_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        for i in range(1, 9):
            rospy.Subscriber(f'/hydrus/thrusters/{i}', Int16,
                             lambda msg, idx=i: self.thruster_callback(msg, idx))

        # Publishers 
        self.depth_pub = rospy.Publisher('/hydrus/depth', Int16, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.thruster_pubs = {
            i: rospy.Publisher(f'/hydrus/thrusters/{i}', Int16, queue_size=10)
            for i in range(1, 9)
        }

        # Action client
        self.controller_client = SimpleActionClient('controller_action', NavigateToWaypointAction)

    def publish_mock_data(self):
        # Publish mock depth
        self.depth_pub.publish(Int16(data=25))  
        rospy.loginfo("Mock depth published.")

        # Publish mock IMU
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.orientation_covariance[0] = -1  # uncalibrated for test
        self.imu_pub.publish(imu_msg)
        rospy.loginfo("Mock IMU published.")

        # Publish mock thruster values
        for i in range(1, 9):
            self.thruster_pubs[i].publish(Int16(data=1500))
        rospy.loginfo("Mock thruster values published.")

    def depth_callback(self, msg):
        self.depth_received = True
        self.last_depth = msg.data

    def imu_callback(self, msg):
        self.imu_received = True

    def thruster_callback(self, msg, idx):
        self.thruster_values[idx] = msg.data

    def check_all(self):
        rospy.loginfo("Checking controller action server...")
        self.controller_client.wait_for_server(timeout=rospy.Duration(5))
        controller_up = self.controller_client.wait_for_server(rospy.Duration(1.0))

        rospy.sleep(2)  

        print("\n==== PREQUALIFICATION TEST RESULTS ====")
        print(f"[{'✓' if self.depth_received else '✗'}] Depth sensor data received")
        print(f"[{'✓' if self.imu_received else '✗'}] IMU data received")
        print(f"[{'✓' if controller_up else '✗'}] Controller action server online")

        thruster_ok = all(abs(val - 1500) < 10 for val in self.thruster_values.values()) and len(self.thruster_values) == 8
        print(f"[{'✓' if thruster_ok else '✗'}] All thrusters neutral")

        if not thruster_ok:
            for i in range(1, 9):
                print(f"Thruster {i}: {self.thruster_values.get(i, 'N/A')}")

        print("========================================")

    def run(self):
        rospy.sleep(1)
        self.publish_mock_data()
        rospy.sleep(3)
        self.check_all()


if __name__ == '__main__':
    try:
        tester = PrequalificationTester()
        tester.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Prequalification tester interrupted.")

#How to Run code (Needs to be verified): 
#roscore
#rosrun your_package prequalification_tester.py
