from vnpy import EzAsyncData


class IMU:
    def __init__(self, com_port="/dev/ttyUSB0", baudrate=115200):
        self.com_port = com_port
        self.baudrate = baudrate
        try:
            self.imu = EzAsyncData.connect(self.com_port, self.baudrate)
        except:
            print("Communication with IMU failed")
            return

    def grab_ypr(self):
        while self.imu.current_data.yaw_pitch_roll is None:
            pass
        self.ypr = self.imu.current_data.yaw_pitch_roll
        return self.ypr


if __name__ == "__main__":
    imu = IMU(com_port="/dev/ttyUSB0", baudrate=115200)
    while True:
        imu.grab_ypr()
        print(imu.ypr.x)
        print(imu.ypr.y)
        print(imu.ypr.z)

    imu.imu.disconnect()
