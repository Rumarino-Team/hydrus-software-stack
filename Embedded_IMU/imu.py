#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_conversions
import tf2_ros

from vnpy import EzAsyncData

class IMUNode:
    def __init__(self, com_port='/dev/ttyUSB0', baudrate=115200):
        """
        This node publishes IMU data and Yaw-Pitch-Roll, plus an Odometry message
        (like the ZED ROS Wrapper publishes /odom).
        """

        # Initialize the node
        rospy.init_node('vn100_imu_node', anonymous=True)

        # ROS Publishers
        self.imu_pub = rospy.Publisher('/vn100/imu', Imu, queue_size=10)
        self.ypr_pub = rospy.Publisher('/vn100/ypr', Vector3, queue_size=10)

        # New: Odometry publisher
        self.odom_pub = rospy.Publisher('/vn100/odom', Odometry, queue_size=10)

        # TF broadcaster if you want to broadcast a transform
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.rate = rospy.Rate(50)  # 50 Hz publishing rate

        # IMU connection
        try:
            self.imu = EzAsyncData.connect(com_port, baudrate)
            rospy.loginfo("Connected to VN-100 IMU")
        except Exception as e:
            rospy.logerr(f"Failed to connect to IMU: {e}")
            return

    def grab_ypr(self):
        """
        Continuously grab IMU data (yaw-pitch-roll) and publish them as:
          - /vn100/imu (sensor_msgs/Imu)
          - /vn100/ypr (geometry_msgs/Vector3)
          - /vn100/odom (nav_msgs/Odometry)   <-- new
        """
        while not rospy.is_shutdown():
            if self.imu.current_data.yaw_pitch_roll is None:
                rospy.logwarn("Waiting for YPR data...")
                self.rate.sleep()
                continue  # Skip if no data

            # ---- Get current YPR from VN sensor
            ypr = self.imu.current_data.yaw_pitch_roll  # x=Yaw, y=Pitch, z=Roll

            # -------------------------------------------------------
            #  1) Publish IMU message
            # -------------------------------------------------------
            imu_msg = Imu()
            now = rospy.Time.now()
            imu_msg.header.stamp = now
            imu_msg.header.frame_id = "imu_link"

            # We have only YPR from the example, so let's convert YPR -> quaternion
            # VectorNav uses x=Yaw, y=Pitch, z=Roll in degrees by default, check your sensor settings
            yaw_rad   = ypr.x * 3.14159 / 180.0
            pitch_rad = ypr.y * 3.14159 / 180.0
            roll_rad  = ypr.z * 3.14159 / 180.0
            quat = tf_conversions.transformations.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)

            imu_msg.orientation.x = quat[0]
            imu_msg.orientation.y = quat[1]
            imu_msg.orientation.z = quat[2]
            imu_msg.orientation.w = quat[3]

            # For a real IMU, fill these from sensor data or set to covariance estimates
            imu_msg.orientation_covariance[0] = -1  # if unknown, set first element to -1
            # If you do have gyro/acc, you can fill them here:
            #    imu_msg.angular_velocity.x    = ...
            #    imu_msg.linear_acceleration.x = ...
            #    etc.

            self.imu_pub.publish(imu_msg)

            # -------------------------------------------------------
            #  2) Publish separate YPR Vector (for quick debug)
            # -------------------------------------------------------
            ypr_msg = Vector3()
            ypr_msg.x = ypr.x   # Yaw
            ypr_msg.y = ypr.y   # Pitch
            ypr_msg.z = ypr.z   # Roll
            self.ypr_pub.publish(ypr_msg)

            # -------------------------------------------------------
            #  3) Publish Odom message (like ZED does)
            # -------------------------------------------------------
            # Here we do not integrate position from the IMU. We just set it to 0
            # If your VN IMU provides integrated position or you have some position logic,
            # fill them accordingly.
            odom_msg = Odometry()
            odom_msg.header.stamp = now
            odom_msg.header.frame_id = "odom"       # The reference frame
            odom_msg.child_frame_id = "base_link"   # The moving frame

            # Position
            odom_msg.pose.pose.position.x = 0.0
            odom_msg.pose.pose.position.y = 0.0
            odom_msg.pose.pose.position.z = 0.0

            # Orientation (same as from the IMU)
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]

            # If you have a state estimator that gives you linear and angular velocities,
            # fill them here. For example:
            #   odom_msg.twist.twist.linear.x = ...
            #   odom_msg.twist.twist.angular.z = ...
            # Otherwise, leave them zero.

            self.odom_pub.publish(odom_msg)

            # -------------------------------------------------------
            #  4) (Optional) Publish TF from odom -> base_link
            # -------------------------------------------------------
            # Because the ZED Wrapper also broadcasts transforms.
            # If you want a TF that is consistent with /odom, do:
            import geometry_msgs.msg
            odom_tf = geometry_msgs.msg.TransformStamped()
            odom_tf.header.stamp = now
            odom_tf.header.frame_id = "odom"
            odom_tf.child_frame_id  = "base_link"
            odom_tf.transform.translation.x = 0.0
            odom_tf.transform.translation.y = 0.0
            odom_tf.transform.translation.z = 0.0
            odom_tf.transform.rotation.x = quat[0]
            odom_tf.transform.rotation.y = quat[1]
            odom_tf.transform.rotation.z = quat[2]
            odom_tf.transform.rotation.w = quat[3]

            self.tf_broadcaster.sendTransform(odom_tf)

            rospy.loginfo_throttle(2.0, f"Published IMU and Odom (YPR: {ypr.x}, {ypr.y}, {ypr.z})")

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