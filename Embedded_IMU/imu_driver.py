#!/usr/bin/env python3

#ros implementation in imu code

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from vnpy import EzAsyncData

class IMUNode:
    def __init__(self, com_port='/dev/ttyUSB0', baudrate=115200):
        rospy.init_node('vn100_imu_node', anonymous=True)

        # ROS Publisher
        self.imu_pub = rospy.Publisher('/vn100/imu', Imu, queue_size=10)
        self.ypr_pub = rospy.Publisher('/vn100/ypr', Vector3, queue_size=10)

        self.rate = rospy.Rate(50)  # 50 Hz publishing rate

        # IMU connection
        try:
            self.imu = EzAsyncData.connect(com_port, baudrate)
            rospy.loginfo("Connected to VN-100 IMU")
        except Exception as e:
            rospy.logerr(f"Failed to connect to IMU: {e}")
            return

    def grab_ypr(self):
        while not rospy.is_shutdown():
            if self.imu.current_data.yaw_pitch_roll is None:
                rospy.logwarn("Waiting for YPR data...")
                continue  # Skip if no data

            ypr = self.imu.current_data.yaw_pitch_roll

            # Create IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = "imu_link"

            imu_msg.orientation.x = ypr.x
            imu_msg.orientation.y = ypr.y
            imu_msg.orientation.z = ypr.z
            imu_msg.orientation.w = 0.0  # Placeholder

            # Create YPR message (for debugging)
            ypr_msg = Vector3()
            ypr_msg.x = ypr.x
            ypr_msg.y = ypr.y
            ypr_msg.z = ypr.z

            # Publish messages
            self.imu_pub.publish(imu_msg)
            self.ypr_pub.publish(ypr_msg)

            rospy.loginfo(f"Published IMU YPR: {ypr.x}, {ypr.y}, {ypr.z}")

            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down IMU node...")
        self.imu.disconnect()

if __name__ == '__main__':
    imu_node = IMUNode()
    try:
        imu_node.grab_ypr()
    except rospy.ROSInterruptException:
        pass
    finally:
        imu_node.shutdown()