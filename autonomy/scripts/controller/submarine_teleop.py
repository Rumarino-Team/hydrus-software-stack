#!/usr/bin/env python3
# filepath: /home/cesar/Projects/hydrus-software-stack/autonomy/scripts/controller/submarine_teleop.py
import os
import signal
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass, field

import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int16  # ← Changed to Int16 for PWM values
from termcolor import colored


# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURACIÓN
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class TeleopConfig:
    # PWM settings for thruster control
    PWM_NEUTRAL: int = 1500  # Neutral position
    PWM_MIN: int = 1300  # Max reverse
    PWM_MAX: int = 1700  # Max forward
    PWM_STEP: int = 50  # Step size for PWM adjustment
    RATE_HZ: int = 10


class SubmarineTeleop:
    # ───────────────────────────────────────────────────────────────────────
    def __init__(self, cfg: TeleopConfig):
        self.cfg = cfg
        rospy.init_node("submarine_teleop_with_visualizer", anonymous=True)

        # Publishers changed to Int16 for PWM values
        self.thruster_pubs = {
            0: rospy.Publisher("/hydrus/thrusters/1", Int16, queue_size=10),
            1: rospy.Publisher("/hydrus/thrusters/2", Int16, queue_size=10),
            2: rospy.Publisher("/hydrus/thrusters/3", Int16, queue_size=10),
            3: rospy.Publisher("/hydrus/thrusters/4", Int16, queue_size=10),
        }
        self.depth_pub = rospy.Publisher("/hydrus/depth", Int16, queue_size=10)
        self.torpedo_pub = rospy.Publisher("/hydrus/torpedo", Int16, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher(
            "/submarine/cmd_vel", TwistStamped, queue_size=10
        )

        # State: PWM values that we send directly to motors (1300-1700)
        self.pwm_values = [self.cfg.PWM_NEUTRAL] * 8

        # Motor mappings - FIXED: Now using 0-based indexing (0-7 for pwm_values list)
        self.front_motors = [0, 4]  # Was [1, 5]
        self.back_motors = [3, 7]  # Was [4, 8] - 8 was causing index error!
        self.depth_motors = [1, 6]  # Was [2, 7]
        self.torpedo_motors = [2, 5]  # Was [3, 6]

        # Visualization state
        self.depth_value = self.cfg.PWM_NEUTRAL
        self.torpedo_value = self.cfg.PWM_NEUTRAL
        self.display_update_interval = 0.2  # seconds

        # Define thruster types for color coding (1-based for display)
        self.regular_thrusters = [1, 4, 5, 8]  # Regular thrusters (cyan)
        self.depth_thrusters = [3, 6]  # Depth thrusters (green)
        self.torpedo_thrusters = [2, 7]  # Torpedo thrusters (yellow)

        self.running = True
        self.key_thread = None
        self.display_thread = None

    # ───────────────────────────────────────────────────────────────────────
    def start(self):
        self._print_help()

        # Start keyboard input thread
        self.key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self.key_thread.start()

        # Start visualization thread
        self.display_thread = threading.Thread(target=self._display_loop, daemon=True)
        self.display_thread.start()

        r = rospy.Rate(self.cfg.RATE_HZ)
        try:
            while not rospy.is_shutdown() and self.running:
                self._publish_all()
                r.sleep()
        finally:
            self._shutdown()

    # ────────────────────────  KEYBOARD HANDLING  ──────────────────────────
    def _print_help(self):
        # Just print a brief header, main display will be handled by visualizer
        print(
            colored(
                "=== SUBMARINE TELEOPERATION WITH LIVE VISUALIZATION ===",
                "blue",
                attrs=["bold"],
            )
        )
        print(
            colored(
                "Controls: W/S=fwd/back | A/D=yaw | I/K=up/down | Space=stop | Q=quit",
                "cyan",
            )
        )
        print(colored("Starting in 2 seconds...", "yellow"))
        time.sleep(2)

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
        self.pwm_values[idx] = max(
            self.cfg.PWM_MIN, min(self.cfg.PWM_MAX, self.pwm_values[idx] + delta)
        )

    def _handle_key(self, k):
        if k == "q":
            self.running = False
            rospy.signal_shutdown("User exit")
        elif k == " ":
            self.pwm_values = [self.cfg.PWM_NEUTRAL] * 8
            # Status will be shown in visualization
        elif k == "w":
            # Forward motion: front thrusters reverse (negative PWM delta), back thrusters forward (positive PWM delta)
            for m in self.front_motors:
                self._adjust_pwm(m, -self.cfg.PWM_STEP)  # FIXED: No more m-1 needed
            for m in self.back_motors:
                self._adjust_pwm(m, self.cfg.PWM_STEP)  # FIXED: No more m-1 needed
        elif k == "s":
            # Backward motion: front thrusters forward (positive PWM delta), back thrusters reverse (negative PWM delta)
            for m in self.front_motors:
                self._adjust_pwm(m, self.cfg.PWM_STEP)  # FIXED: No more m-1 needed
            for m in self.back_motors:
                self._adjust_pwm(m, -self.cfg.PWM_STEP)  # FIXED: No more m-1 needed
        elif k == "a":
            for m in self.front_motors:
                self._adjust_pwm(m, -self.cfg.PWM_STEP)  # FIXED: No more m-1 needed
            for m in self.back_motors:
                self._adjust_pwm(m, self.cfg.PWM_STEP)  # FIXED: No more m-1 needed
        elif k == "d":
            for m in self.front_motors:
                self._adjust_pwm(m, self.cfg.PWM_STEP)  # FIXED: No more m-1 needed
            for m in self.back_motors:
                self._adjust_pwm(m, -self.cfg.PWM_STEP)  # FIXED: No more m-1 needed
        elif k == "i":
            for m in self.depth_motors:
                self._adjust_pwm(m, self.cfg.PWM_STEP)  # FIXED: No more m-1 needed
        elif k == "k":
            for m in self.depth_motors:
                self._adjust_pwm(m, -self.cfg.PWM_STEP)  # FIXED: No more m-1 needed

    # ──────────────────────────  PUBLISHERS  ───────────────────────────────
    def _publish_all(self):
        # thrusters 1-4
        for i in range(4):
            self.thruster_pubs[i].publish(Int16(self.pwm_values[i]))

        # depth motors - send average of depth motor PWM values
        self.depth_value = int(
            sum(self.pwm_values[m] for m in self.depth_motors) / len(self.depth_motors)
        )
        self.depth_pub.publish(Int16(self.depth_value))

        # torpedo motors - send average of torpedo motor PWM values
        self.torpedo_value = int(
            sum(self.pwm_values[m] for m in self.torpedo_motors)
            / len(self.torpedo_motors)
        )
        self.torpedo_pub.publish(Int16(self.torpedo_value))

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
        fwd_pwm = (
            sum(self.pwm_values[m] for m in self.front_motors + self.back_motors) / 4.0
        )
        tw.twist.linear.x = (fwd_pwm - self.cfg.PWM_NEUTRAL) / pwm_range

        # Depth velocity - average of depth motors
        depth_pwm = sum(self.pwm_values[m] for m in self.depth_motors) / 2.0
        tw.twist.linear.z = (depth_pwm - self.cfg.PWM_NEUTRAL) / pwm_range

        # Angular velocity - difference between left and right side motors
        left_pwm = (
            sum(self.pwm_values[m] for m in [self.front_motors[0], self.back_motors[0]])
            / 2.0
        )
        right_pwm = (
            sum(self.pwm_values[m] for m in [self.front_motors[1], self.back_motors[1]])
            / 2.0
        )
        tw.twist.angular.z = (right_pwm - left_pwm) / pwm_range

        self.cmd_vel_pub.publish(tw)

    # ─────────────────────────  SHUTDOWN  ───────────────────────────────────
    def _shutdown(self):
        self.running = False
        self.pwm_values = [self.cfg.PWM_NEUTRAL] * 8
        self._publish_all()

        # Clear screen and show exit message
        os.system("clear")
        print(
            colored("\n=== SUBMARINE TELEOPERATION SHUTDOWN ===", "red", attrs=["bold"])
        )
        print(colored("All thrusters stopped (PWM = 1500)", "yellow"))
        print(colored("Exiting submarine teleoperation…", "blue"))

        # Wait a moment for threads to finish
        time.sleep(0.5)

    # ────────────────────────  VISUALIZATION  ──────────────────────────────
    def _display_loop(self):
        """Continuously update the ASCII diagram with current values"""
        while self.running and not rospy.is_shutdown():
            self._display_thrusters()
            time.sleep(self.display_update_interval)

    def _get_thruster_str(self, thruster_id):
        """Format a thruster value with appropriate color and direction indicator"""
        # Convert from 0-based internal index to 1-based display index
        if thruster_id <= 4:
            value = self.pwm_values[thruster_id - 1]  # Thrusters 1-4
        else:
            # For thrusters 5-8, use the extended pwm_values or calculated values
            if thruster_id == 5:
                value = (
                    self.pwm_values[4]
                    if len(self.pwm_values) > 4
                    else self.pwm_values[0]
                )
            elif thruster_id == 6:
                value = self.depth_value
            elif thruster_id == 7:
                value = self.torpedo_value
            elif thruster_id == 8:
                value = (
                    self.pwm_values[7]
                    if len(self.pwm_values) > 7
                    else self.pwm_values[3]
                )
            else:
                value = self.cfg.PWM_NEUTRAL

        # Determine direction indicator
        direction = "○"  # Neutral
        if value > 1510:
            direction = "►"  # Forward
        elif value < 1490:
            direction = "◄"  # Reverse

        # Format string with PWM value
        value_str = f"{thruster_id}{direction} [{value}]"

        # Apply appropriate color based on thruster type
        if thruster_id in self.regular_thrusters:
            return colored(value_str, "cyan")
        elif thruster_id in self.depth_thrusters:
            return colored(value_str, "green")
        else:  # torpedo thrusters
            return colored(value_str, "yellow")

    def _display_thrusters(self):
        """Display the ASCII diagram with current thruster values"""
        # Clear the terminal
        os.system("clear")

        # Print header
        print(
            colored(
                "=== HYDRUS SUBMARINE TELEOPERATION + VISUALIZATION ===",
                "white",
                attrs=["bold"],
            )
        )
        print(
            colored("Controls: ", "white")
            + colored("W/S", "green")
            + colored("=fwd/back | ", "white")
            + colored("A/D", "green")
            + colored("=yaw | ", "white")
            + colored("I/K", "green")
            + colored("=up/down | ", "white")
            + colored("Space", "red")
            + colored("=stop | ", "white")
            + colored("Q", "red")
            + colored("=quit", "white")
        )
        print(
            colored("Regular thrusters: ", "cyan")
            + colored("Depth thrusters: ", "green")
            + colored("Torpedo thrusters: ", "yellow")
        )
        print(
            colored(f"Depth command: {self.depth_value}", "green")
            + colored(f" | Torpedo command: {self.torpedo_value}", "yellow")
        )
        print("")

        # Generate ASCII diagram with thruster values
        diagram = [
            f"{self._get_thruster_str(1)}           {self._get_thruster_str(5)}",
            "     \\          /",
            "      |________|        ",
            f"{self._get_thruster_str(2)}--|        |--{self._get_thruster_str(6)}",
            "      |        |",
            "      |        |",
            f"{self._get_thruster_str(3)}--|________|--{self._get_thruster_str(7)}",
            "      |        |",
            f"{self._get_thruster_str(4)}/          \\{self._get_thruster_str(8)}",
        ]

        # Print diagram
        for line in diagram:
            print(line)

        # Show current PWM values for debugging
        print("\n" + colored("Current PWM Values:", "white", attrs=["bold"]))
        for i in range(4):
            print(colored(f"T{i+1}: {self.pwm_values[i]}", "cyan"), end="  ")
        print()


# ─────────────────────────────────────────────────────────────────────────────
def _signal_handler(_, __):
    print(colored("\nSIGINT/SIGTERM received", "red"))
    rospy.signal_shutdown("Signal received")


def main():
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)
    SubmarineTeleop(TeleopConfig()).start()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        # restaura terminal si crashea con tty raw
        try:
            termios.tcsetattr(
                sys.stdin.fileno(),
                termios.TCSADRAIN,
                termios.tcgetattr(sys.stdin.fileno()),
            )
        except termios.error:
            pass
        print(f"Error: {e}")
