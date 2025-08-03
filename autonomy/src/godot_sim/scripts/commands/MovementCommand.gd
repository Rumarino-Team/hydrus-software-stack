# commands/MovementCommand.gd
# Represents a movement command that can be executed by the thruster system
class_name MovementCommand

enum Type {
	FORWARD,
	BACKWARD,
	TURN_LEFT,
	TURN_RIGHT,
	MOVE_UP,
	MOVE_DOWN,
	STOP
}

var type: Type
var intensity: float  # 0.0 to 1.0

func _init(movement_type: Type, movement_intensity: float = 1.0):
	type = movement_type
	intensity = clamp(movement_intensity, 0.0, 1.0)

static func forward(intensity: float = 1.0) -> MovementCommand:
	return MovementCommand.new(Type.FORWARD, intensity)

static func backward(intensity: float = 1.0) -> MovementCommand:
	return MovementCommand.new(Type.BACKWARD, intensity)

static func turn_left(intensity: float = 1.0) -> MovementCommand:
	return MovementCommand.new(Type.TURN_LEFT, intensity)

static func turn_right(intensity: float = 1.0) -> MovementCommand:
	return MovementCommand.new(Type.TURN_RIGHT, intensity)

static func move_up(intensity: float = 1.0) -> MovementCommand:
	return MovementCommand.new(Type.MOVE_UP, intensity)

static func move_down(intensity: float = 1.0) -> MovementCommand:
	return MovementCommand.new(Type.MOVE_DOWN, intensity)

static func stop() -> MovementCommand:
	return MovementCommand.new(Type.STOP, 0.0)
