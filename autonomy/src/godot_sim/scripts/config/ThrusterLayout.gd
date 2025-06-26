# config/ThrusterLayout.gd
# Defines the thruster configuration and movement patterns
class_name ThrusterLayout

# Thruster IDs for different movement types
var movement_thrusters = [1, 4, 5, 8]  # Angled thrusters for forward/backward/turning
var depth_thrusters = [2, 7]           # Vertical thrusters for up/down
var torpedo_thrusters = [3, 6]         # Horizontal thrusters for side movement

func move_forward(thruster_system: ThrusterSystem, intensity: float) -> void:
	var pwm_value = 1500 + (intensity * 100)  # intensity should be 0-1
	for thruster_id in movement_thrusters:
		thruster_system.set_thruster_pwm(thruster_id, pwm_value)

func move_backward(thruster_system: ThrusterSystem, intensity: float) -> void:
	var pwm_value = 1500 - (intensity * 100)
	for thruster_id in movement_thrusters:
		thruster_system.set_thruster_pwm(thruster_id, pwm_value)

func turn_left(thruster_system: ThrusterSystem, intensity: float) -> void:
	var pwm_forward = 1500 - (intensity * 100)
	var pwm_backward = 1500 + (intensity * 100)
	# Left side thrusters backward, right side forward
	thruster_system.set_thruster_pwm(1, pwm_forward)   # Front left
	thruster_system.set_thruster_pwm(4, pwm_forward)   # Back left
	thruster_system.set_thruster_pwm(5, pwm_backward)  # Front right
	thruster_system.set_thruster_pwm(8, pwm_backward)  # Back right

func turn_right(thruster_system: ThrusterSystem, intensity: float) -> void:
	var pwm_forward = 1500 + (intensity * 100)
	var pwm_backward = 1500 - (intensity * 100)
	# Right side thrusters backward, left side forward
	thruster_system.set_thruster_pwm(1, pwm_forward)   # Front left
	thruster_system.set_thruster_pwm(4, pwm_forward)   # Back left
	thruster_system.set_thruster_pwm(5, pwm_backward)  # Front right
	thruster_system.set_thruster_pwm(8, pwm_backward)  # Back right

func move_up(thruster_system: ThrusterSystem, intensity: float) -> void:
	var pwm_value = 1500 + (intensity * 100)
	for thruster_id in depth_thrusters:
		thruster_system.set_thruster_pwm(thruster_id, pwm_value)

func move_down(thruster_system: ThrusterSystem, intensity: float) -> void:
	var pwm_value = 1500 - (intensity * 100)
	for thruster_id in depth_thrusters:
		thruster_system.set_thruster_pwm(thruster_id, pwm_value)

func fire_torpedo(thruster_system: ThrusterSystem, side: String, intensity: float) -> void:
	var pwm_value = 1500 + (intensity * 100)
	if side == "left":
		thruster_system.set_thruster_pwm(3, pwm_value)
	elif side == "right":
		thruster_system.set_thruster_pwm(6, pwm_value)

func get_thruster_configuration() -> Dictionary:
	return {
		"movement": movement_thrusters,
		"depth": depth_thrusters,
		"torpedo": torpedo_thrusters
	}
