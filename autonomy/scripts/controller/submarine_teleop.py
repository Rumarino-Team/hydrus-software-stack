#!/usr/bin/env python3
# filepath: /home/cesar/Projects/hydrus-software-stack/autonomy/scripts/controller/submarine_teleop.py
import sys, tty, termios, threading, signal
import rospy
from std_msgs.msg import Float32          # ← enviamos PWM
from geometry_msgs.msg import TwistStamped
from termcolor import colored
from dataclasses import dataclass, field


# ─────────────────────────────────────────────────────────────────────────────
#  CONFIGURACIÓN
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class TeleopConfig:
    # Traducción de niveles de velocidad a PWM (tu tabla)
    SPEED_TRANSLATION: dict = field(default_factory=lambda: {
        0: 1500,   # neutral
        1: 1550,
        2: 1600,
        3: 1650,
        4: 1700,
        -1: 1450,
        -2: 1400,
        -3: 1350,
        -4: 1300,
    })
    MAX_LEVEL: int   =  4        # +4 → 1700
    MIN_LEVEL: int   = -4        # –4 → 1300
    KEY_STEP: int    =  1        # cuánto cambia por pulsación
    RATE_HZ: int     = 10


class SubmarineTeleop:
    # ───────────────────────────────────────────────────────────────────────
    def __init__(self, cfg: TeleopConfig):
        self.cfg = cfg
        rospy.init_node("submarine_teleop", anonymous=True)

        # Publishers renombrados
        self.thruster_pubs = {
            0: rospy.Publisher("/hydrus/thrusters/1", Float32, queue_size=10),
            1: rospy.Publisher("/hydrus/thrusters/2", Float32, queue_size=10),
            2: rospy.Publisher("/hydrus/thrusters/3", Float32, queue_size=10),
            3: rospy.Publisher("/hydrus/thrusters/4", Float32, queue_size=10),
        }
        self.depth_pub   = rospy.Publisher("/hydrus/depth",   Float32, queue_size=10)
        self.torpedo_pub = rospy.Publisher("/hydrus/torpedo", Float32, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher("/submarine/cmd_vel",
                                           TwistStamped, queue_size=10)

        # Estado: niveles (-4…4) que luego convertimos a PWM con SPEED_TRANSLATION
        self.levels = [0] * 8

        # Mapeos de motores
        self.front_motors   = [0, 4]   # 1,5
        self.back_motors    = [3, 7]   # 4,8
        self.depth_motors   = [1, 6]   # 2,7
        self.torpedo_motors = [2, 5]   # 3,6

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
    def _clamp(self, idx, delta):
        self.levels[idx] = max(self.cfg.MIN_LEVEL,
                               min(self.cfg.MAX_LEVEL, self.levels[idx] + delta))

    def _handle_key(self, k):
        if k == "q":
            self.running = False
            rospy.signal_shutdown("User exit")
        elif k == " ":
            self.levels = [0] * 8
            print(colored("STOP", "red"))
        elif k == "w":
            for m in self.front_motors + self.back_motors: self._clamp(m,  self.cfg.KEY_STEP)
            print(colored("FORWARD", "green"))
        elif k == "s":
            for m in self.front_motors + self.back_motors: self._clamp(m, -self.cfg.KEY_STEP)
            print(colored("BACKWARD", "green"))
        elif k == "a":
            for m in self.front_motors: self._clamp(m, -self.cfg.KEY_STEP)
            for m in self.back_motors:  self._clamp(m,  self.cfg.KEY_STEP)
            print(colored("YAW LEFT", "green"))
        elif k == "d":
            for m in self.front_motors: self._clamp(m,  self.cfg.KEY_STEP)
            for m in self.back_motors:  self._clamp(m, -self.cfg.KEY_STEP)
            print(colored("YAW RIGHT", "green"))
        elif k == "i":
            for m in self.depth_motors: self._clamp(m,  self.cfg.KEY_STEP)
            print(colored("UP", "green"))
        elif k == "k":
            for m in self.depth_motors: self._clamp(m, -self.cfg.KEY_STEP)
            print(colored("DOWN", "green"))

    # ──────────────────────────  PUBLISHERS  ───────────────────────────────
    def _level_to_pwm(self, lvl):
        return self.cfg.SPEED_TRANSLATION.get(lvl, 1500)

    def _publish_all(self):
        # thrusters 1-4
        for i in range(4):
            pwm = self._level_to_pwm(self.levels[i])
            self.thruster_pubs[i].publish(Float32(pwm))

        # depth motores 5-6 → un único tópico
        depth_pwm = sum(self._level_to_pwm(self.levels[m]) for m in self.depth_motors) / 2.0
        self.depth_pub.publish(Float32(depth_pwm))

        # torpedo motores 7-8
        torp_pwm = sum(self._level_to_pwm(self.levels[m]) for m in self.torpedo_motors) / 2.0
        self.torpedo_pub.publish(Float32(torp_pwm))

        # cmd_vel (aprox.)
        self._publish_cmd_vel()

    def _publish_cmd_vel(self):
        tw = TwistStamped()
        tw.header.stamp = rospy.Time.now()
        tw.header.frame_id = "base_link"

        fwd = sum(self.levels[m] for m in self.front_motors + self.back_motors) / 4.0
        tw.twist.linear.x = fwd / self.cfg.MAX_LEVEL

        depth = sum(self.levels[m] for m in self.depth_motors) / 2.0
        tw.twist.linear.z = depth / self.cfg.MAX_LEVEL

        left  = sum(self.levels[m] for m in [self.front_motors[0], self.back_motors[0]])
        right = sum(self.levels[m] for m in [self.front_motors[1], self.back_motors[1]])
        tw.twist.angular.z = (right - left) / (2 * self.cfg.MAX_LEVEL)

        self.cmd_vel_pub.publish(tw)

    # ─────────────────────────  SHUTDOWN  ───────────────────────────────────
    def _shutdown(self):
        self.levels = [0] * 8
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
