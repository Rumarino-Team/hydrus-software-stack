#!/usr/bin/env python3
"""
ASCII Visualizer for Gate-Mission Tester
• ⊕  = submarine
• ⊓  = gate centre
• ·  = recent path (last 50 points)
• +  = world origin
"""

import os, math, rospy
from geometry_msgs.msg import PoseStamped

class GateVisualizer:
    def __init__(self):
        rospy.init_node("gate_visualizer")

        # dynamic items
        self.submarine_pose = None                  # (x,y,z) updated by callback
        self.gate_position  = self.get_gate_param() # tuple(x,y,z)
        self.path           = []                    # last 50 poses

        # fixed display geometry
        self.width, self.height = 80, 30
        self.update_scale()

        # ROS subscribers
        rospy.Subscriber("/submarine_pose", PoseStamped, self.pose_cb)

    # ---------------------------------------------------------------------
    # ROS helpers
    # ---------------------------------------------------------------------
    @staticmethod
    def get_gate_param():
        """Return gate centre from parameter server, default (10,10,10)."""
        return tuple(rospy.get_param("/gate_position", [10.0, 3.0, 10.0]))

    def pose_cb(self, msg):
        """Handle incoming submarine pose, store & append to path ring-buffer."""
        self.submarine_pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        self.path.append(self.submarine_pose)
        if len(self.path) > 50:         # keep recent trail only
            self.path.pop(0)

    # ---------------------------------------------------------------------
    # Drawing helpers
    # ---------------------------------------------------------------------
    def update_scale(self):
        """Compute metres-per-character so that gate fits nicely in view."""
        max_coord = max(abs(self.gate_position[0]), abs(self.gate_position[1]), 1e-6)
        self.scale = max_coord / (min(self.width, self.height) * 0.4)

    def world_to_screen(self, x, y, z):
        """Project world (x,y,z) onto ASCII grid indices (col,row)."""
        sx = int(self.width  / 2 + x / self.scale)
        sy = int(self.height / 2 - y / self.scale)          # y axis up on screen
        return (min(max(sx, 0), self.width  - 1),
                min(max(sy, 0), self.height - 1))

    def clear(self):
        os.system("clear" if os.name == "posix" else "cls")

    # ---------------------------------------------------------------------
    # Main drawing loop
    # ---------------------------------------------------------------------
    def draw_frame(self):
        if not self.submarine_pose:                 # nothing to draw yet
            return

        # pull latest gate centre & rescale if it changed
        new_gate = self.get_gate_param()
        if new_gate != self.gate_position:
            self.gate_position = new_gate
            self.update_scale()

        # blank canvas
        grid = [[" " for _ in range(self.width)] for _ in range(self.height)]

        # plot path
        for pos in self.path:
            x, y = self.world_to_screen(*pos)
            grid[y][x] = "·"

        # plot gate
        gx, gy = self.world_to_screen(*self.gate_position)
        grid[gy][gx] = "⊓"

        # plot submarine
        sx, sy = self.world_to_screen(*self.submarine_pose)
        grid[sy][sx] = "⊕"

        # axes
        cx, cy = self.width//2, self.height//2
        for i in range(self.width):  grid[cy][i] = "─"
        for i in range(self.height): grid[i][cx] = "│"
        grid[cy][cx] = "+"

        # render
        self.clear()
        border = "+" + "-"*self.width + "+"
        print(border)
        for row in grid:
            print("|" + "".join(row) + "|")
        print(border)

        # status
        dx = self.submarine_pose[0] - self.gate_position[0]
        dy = self.submarine_pose[1] - self.gate_position[1]
        dz = self.submarine_pose[2] - self.gate_position[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        print(f"\nLegend: ⊕ submarine | ⊓ gate | · path | + origin")
        print(f"Scale : 1 char ≈ {self.scale:4.1f} m")
        print(f"Sub   : ({self.submarine_pose[0]:5.2f}, "
                         f"{self.submarine_pose[1]:5.2f}, "
                         f"{self.submarine_pose[2]:5.2f})")
        print(f"Gate  : {self.gate_position}")
        print(f"Δdist : {dist:5.2f} m (centre-to-centre)")
        print("Ctrl-C to quit")

    # ---------------------------------------------------------------------
    # Run
    # ---------------------------------------------------------------------
    def run(self):
        rate = rospy.Rate(10)            # 10 Hz refresh
        while not rospy.is_shutdown():
            self.draw_frame()
            rate.sleep()

# -------------------------------------------------------------------------
if __name__ == "__main__":
    try:
        GateVisualizer().run()
    except rospy.ROSInterruptException:
        pass
