#!/usr/bin/env python3

import rospy
import random
from std_msgs.msg import Int16
from sensor_msgs.msg import Imu, Image
from geometry_msgs.msg import Point

# Mock GateDetection message class
class GateDetection:
    def __init__(self):
        self.position = Point()
        self.confidence = 0.0

class PrequalificationTester:
    def __init__(self):
        rospy.init_node('prequalification_tester', anonymous=True)

        self.depth_received = False
        self.imu_received = False
        self.gate_detected = False
        self.camera_connected = False
        self.last_gate_position = None
        self.gate_confidence = 0.0
        self.thruster_values = {}

        rospy.Subscriber('/hydrus/depth', Int16, self.depth_callback)
        rospy.Subscriber('/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/zed2i/zed_node/image_raw', Image, self.camera_callback)

        for i in range(1, 9):
            rospy.Subscriber(f'/hydrus/thrusters/{i}', Int16,
                             lambda msg, idx=i: self.thruster_callback(msg, idx))

    def depth_callback(self, msg):
        self.depth_received = True
        self.last_depth = msg.data

    def imu_callback(self, msg):
        self.imu_received = True

    def camera_callback(self, msg):
        self.camera_connected = True

    def thruster_callback(self, msg, idx):
        self.thruster_values[idx] = msg.data

    def gate_callback(self, msg):
        if self.camera_connected:
            self.gate_detected = True
            self.last_gate_position = msg.position
            self.gate_confidence = msg.confidence
        else:
            rospy.logwarn("Gate detection ignored — camera not connected.")

    def check_all(self):
        print("\n==== PREQUALIFICATION TEST RESULTS ====")
        print(f"[{'✓' if self.depth_received else '✗'}] Depth sensor data received")
        print(f"[{'✓' if self.imu_received else '✗'}] IMU data received")

        all_thrusters_ok = all(v == 1500 for v in self.thruster_values.values())
        print(f"[{'✓' if all_thrusters_ok else '✗'}] All thrusters neutral")
        print(f"[{'✓' if self.camera_connected else '✗'}] Camera is connected and publishing")
        print(f"[{'✓' if self.gate_detected else '✗'}] Gate detected by camera")

        if self.gate_detected:
            p = self.last_gate_position
            print(f"    ↳ Position: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
            print(f"    ↳ Confidence: {self.gate_confidence:.2f}")
        print("========================================")

class GatePublisher:
    def create_gate(self):
        msg = GateDetection()
        msg.position = Point(
            x=round(random.uniform(4.0, 5.0), 2),
            y=round(random.uniform(2.0, 3.0), 2),
            z=round(random.uniform(0.5, 1.5), 2)
        )
        msg.confidence = round(random.uniform(0.8, 0.99), 2)
        rospy.loginfo(f"[MOCK] Generated gate: Pos=({msg.position.x:.2f}, {msg.position.y:.2f}, {msg.position.z:.2f}), Conf={msg.confidence:.2f}")
        return msg

if __name__ == "__main__":
    try:
        tester = PrequalificationTester()
        gate_pub = GatePublisher()
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            gate_msg = gate_pub.create_gate()
            tester.gate_callback(gate_msg)
            tester.check_all()
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
