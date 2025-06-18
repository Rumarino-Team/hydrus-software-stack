# interfaces/IThruster.gd
# Interface for thruster components
class_name IThruster

signal thrust_changed(thruster_id: int, thrust_value: float)

var id: int
var position: Vector3
var direction: Vector3
var max_force: float

func _init(thruster_id: int, pos: Vector3, dir: Vector3, max_f: float):
	id = thruster_id
	position = pos
	direction = dir.normalized()
	max_force = max_f

# Abstract methods that must be implemented
func set_pwm(pwm_value: float) -> void:
	assert(false, "set_pwm must be implemented by subclass")

func get_thrust_force() -> Vector3:
	assert(false, "get_thrust_force must be implemented by subclass")
	return Vector3.ZERO

func update_visual() -> void:
	assert(false, "update_visual must be implemented by subclass")

func get_current_pwm() -> float:
	assert(false, "get_current_pwm must be implemented by subclass")
	return 0.0
