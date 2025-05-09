#!/usr/bin/env python3
# filepath: /home/cesar/Projects/hydrus-software-stack/autonomy/scripts/controller/submarine_teleop.py
import sys, tty, termios, threading, signal
import rospy
from std_msgs.msg import Int16          # ← Changed to Int16 for PWM values
from geometry_msgs.msg import TwistStamped
from termcolor import colored
from dataclasses import dataclass, field


# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURACIÓN
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class TeleopConfig:
    # PWM settings for thruster control
    PWM_NEUTRAL: int = 1500       # Neutral position
    PWM_MIN: int = 1300           # Max reverse
    PWM_MAX: int = 1700           # Max forward
    PWM_STEP: int = 50            # Step size for PWM adjustment
    RATE_HZ: int = 10


class SubmarineTeleop:
    # ───────────────────────────────────────────────────────────────────────
    def __init__(self, cfg: TeleopConfig):
        self.cfg = cfg
        rospy.init_node("submarine_teleop", anonymous=True)

        # Publishers changed to Int16 for PWM values
        self.thruster_pubs = {
            0: rospy.Publisher("/hydrus/thrusters/1", Int16, queue_size=10),
            1: rospy.Publisher("/hydrus/thrusters/2", Int16, queue_size=10),
            2: rospy.Publisher("/hydrus/thrusters/3", Int16, queue_size=10),
            3: rospy.Publisher("/hydrus/thrusters/4", Int16, queue_size=10),
        }
        self.depth_pub   = rospy.Publisher("/hydrus/depth",   Int16, queue_size=10)
        self.torpedo_pub = rospy.Publisher("/hydrus/torpedo", Int16, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("/submarine/cmd_vel",
                                           TwistStamped, queue_size=10)

        # State: PWM values that we send directly to motors (1300-1700)
        self.pwm_values = [self.cfg.PWM_NEUTRAL] * 8

        # Mapeos de motores
        self.front_motors   = [1, 5]   # 1,5
        self.back_motors    = [4, 8]   # 4,8
        self.depth_motors   = [2, 7]   # 2,7
        self.torpedo_motors = [3, 6]   # 3,6

        self.running = True
        self.key_thread = None

    # ───────────────────────────────────────────────────────────────────────
    def start(self):
        self._print_help()
        self.key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self.key_thread.start()

        r = rospy.Rate(self.cfg.RATE_HZ)
        try:
            while not rospy.is_shutdown() and self.running:
                self._publish_all()
                r.sleep()
        finally:
            self._shutdown()

    # ────────────────────────  KEYBOARD HANDLING  ──────────────────────────
    def _print_help(self):
        print(colored("\n=== SUBMARINE KEYBOARD TELEOPERATION ===", "blue", attrs=["bold"]))
        print(colored("W/S : forward/back  |  A/D : yaw  |  I/K : up/down", "cyan"))
        print(colored("Space : neutral     |  Q   : quit\n", "cyan"))

    @staticmethod
    def _get_key():
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch.lower()

    def _key_loop(self):
        while self.running:
            self._handle_key(self._get_key())

    # ────────────────────────  MOVEMENT COMMANDS  ─────────────────────────
    def _adjust_pwm(self, idx, delta):
        """Adjust PWM value with boundaries"""
        self.pwm_values[idx] = max(self.cfg.PWM_MIN, 
                               min(self.cfg.PWM_MAX, self.pwm_values[idx] + delta))

    def _handle_key(self, k):
        if k == "q":
            self.running = False
            rospy.signal_shutdown("User exit")
        elif k == " ":
            self.pwm_values = [self.cfg.PWM_NEUTRAL] * 8
            print(colored("STOP", "red"))
        elif k == "w":
            # Forward motion: front thrusters reverse (negative PWM delta), back thrusters forward (positive PWM delta)
            for m in self.front_motors: 
                self._adjust_pwm(m-1, -self.cfg.PWM_STEP)  # Decrease PWM for front thrusters
            for m in self.back_motors:  
                self._adjust_pwm(m-1, self.cfg.PWM_STEP)   # Increase PWM for back thrusters
            print(colored("FORWARD", "green"))
        elif k == "s":
            # Backward motion: front thrusters forward (positive PWM delta), back thrusters reverse (negative PWM delta)
            for m in self.front_motors: 
                self._adjust_pwm(m-1, self.cfg.PWM_STEP)   # Increase PWM for front thrusters
            for m in self.back_motors:  
                self._adjust_pwm(m-1, -self.cfg.PWM_STEP)  # Decrease PWM for back thrusters
            print(colored("BACKWARD", "green"))
        elif k == "a":
            for m in self.front_motors: self._adjust_pwm(m-1, -self.cfg.PWM_STEP)
            for m in self.back_motors:  self._adjust_pwm(m-1, self.cfg.PWM_STEP)
            print(colored("YAW LEFT", "green"))
        elif k == "d":
            for m in self.front_motors: self._adjust_pwm(m-1, self.cfg.PWM_STEP)
            for m in self.back_motors:  self._adjust_pwm(m-1, -self.cfg.PWM_STEP)
            print(colored("YAW RIGHT", "green"))
        elif k == "i":
            for m in self.depth_motors: self._adjust_pwm(m-1, self.cfg.PWM_STEP)
            print(colored("UP", "green"))
        elif k == "k":
            for m in self.depth_motors: self._adjust_pwm(m-1, -self.cfg.PWM_STEP)
            print(colored("DOWN", "green"))

    # ──────────────────────────  PUBLISHERS  ───────────────────────────────
    def _publish_all(self):
        # thrusters 1-4
        for i in range(4):
            self.thruster_pubs[i].publish(Int16(self.pwm_values[i]))

        # depth motors - send average of depth motor PWM values
        depth_pwm = int(sum(self.pwm_values[m] for m in self.depth_motors) / len(self.depth_motors))
        self.depth_pub.publish(Int16(depth_pwm))

        # torpedo motors - send average of torpedo motor PWM values
        torpedo_pwm = int(sum(self.pwm_values[m] for m in self.torpedo_motors) / len(self.torpedo_motors))
        self.torpedo_pub.publish(Int16(torpedo_pwm))

        # cmd_vel (aprox.)
        self._publish_cmd_vel()

    def _publish_cmd_vel(self):
        tw = TwistStamped()
        tw.header.stamp = rospy.Time.now()
        tw.header.frame_id = "base_link"

        # Convert PWM values to normalized values for twist message
        # PWM range: PWM_MIN to PWM_MAX, normalize to -1.0 to 1.0
        pwm_range = self.cfg.PWM_MAX - self.cfg.PWM_NEUTRAL
        
        # Forward velocity - average of front and back motors
        fwd_pwm = sum(self.pwm_values[m] for m in self.front_motors + self.back_motors) / 4.0
        tw.twist.linear.x = (fwd_pwm - self.cfg.PWM_NEUTRAL) / pwm_range
        
        # Depth velocity - average of depth motors
        depth_pwm = sum(self.pwm_values[m] for m in self.depth_motors) / 2.0
        tw.twist.linear.z = (depth_pwm - self.cfg.PWM_NEUTRAL) / pwm_range
        
        # Angular velocity - difference between left and right side motors
        left_pwm = sum(self.pwm_values[m] for m in [self.front_motors[0], self.back_motors[0]]) / 2.0
        right_pwm = sum(self.pwm_values[m] for m in [self.front_motors[1], self.back_motors[1]]) / 2.0
        tw.twist.angular.z = (right_pwm - left_pwm) / pwm_range
        
        self.cmd_vel_pub.publish(tw)

    # ─────────────────────────  SHUTDOWN  ───────────────────────────────────
    def _shutdown(self):
        self.pwm_values = [self.cfg.PWM_NEUTRAL] * 8
        self._publish_all()
        print(colored("\nExiting submarine teleoperation…", "blue"))


# ─────────────────────────────────────────────────────────────────────────────
def _signal_handler(_, __):
    print(colored("\nSIGINT/SIGTERM received", "red"))
    rospy.signal_shutdown("Signal received")


def main():
    signal.signal(signal.SIGINT,  _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)
    SubmarineTeleop(TeleopConfig()).start()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        # restaura terminal si crashea con tty raw
        try:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, termios.tcgetattr(sys.stdin.fileno()))
        except termios.error:
            pass
        print(f"Error: {e}")
