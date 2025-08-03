extends Node3D

# Water environment properties
@export var water_color: Color = Color(0.1, 0.4, 0.8, 0.8)
@export var water_depth: float = 10.0
@export var water_clarity: float = 0.5  # 0-1, affects visibility distance

# Lighting properties
@export var ambient_light_color: Color = Color(0.05, 0.15, 0.3, 1.0)
@export var directional_light_color: Color = Color(0.8, 0.8, 1.0, 1.0)
@export var directional_light_energy: float = 0.6

# References to scene nodes
@onready var water_surface: StaticBody3D = $WaterSurface
@onready var water_volume: Area3D = $WaterVolume
@onready var directional_light: DirectionalLight3D = $DirectionalLight
@onready var ambient_light: WorldEnvironment = get_node("../WorldEnvironment")

# Objects in the environment
var objects: Array = []
var targets: Array = []

func _ready():
	# Initialize water properties
	if water_surface:
		# Get the MeshInstance3D child of the StaticBody3D
		var mesh_instance = water_surface.get_node("MeshInstance")
		if mesh_instance:
			var mat = mesh_instance.get_surface_override_material(0)
			if mat:
				mat.albedo_color = water_color

	# Initialize lighting
	if ambient_light and ambient_light.environment:
		ambient_light.environment.ambient_light_color = ambient_light_color

	if directional_light:
		directional_light.light_color = directional_light_color
		directional_light.light_energy = directional_light_energy

	# Apply water effects to cameras in the scene
	setup_underwater_effects()

	# Add collision detection for water volume
	add_water_collision()

	print("Water environment initialized")

func setup_underwater_effects():
	var cameras = find_cameras(self)
	for camera in cameras:
		var environment = camera.environment
		if environment == null:
			environment = Environment.new()
			camera.environment = environment

		# Set underwater fog
		environment.fog_enabled = true
		environment.fog_color = water_color
		environment.fog_depth_enabled = true
		environment.fog_depth_begin = 1.0
		environment.fog_depth_end = 15.0 * water_clarity

		# Underwater light absorption
		environment.adjustment_enabled = true
		environment.adjustment_brightness = 0.9
		environment.adjustment_contrast = 1.1
		environment.adjustment_saturation = 1.2

		# Slight color shifting
		if not environment.adjustment_color_correction:
			var gradient = Gradient.new()
			gradient.add_point(0.0, Color(0.8, 0.9, 1.0))
			gradient.add_point(1.0, Color(1.0, 1.0, 1.0))

			var gradient_texture = GradientTexture1D.new()
			gradient_texture.gradient = gradient

			environment.adjustment_color_correction = gradient_texture

func find_cameras(node: Node) -> Array:
	var cameras: Array = []
	if node is Camera3D:
		cameras.append(node)
	for child in node.get_children():
		cameras += find_cameras(child)
	return cameras

func add_water_collision():
	if water_volume:
		# The water_volume is already an Area3D, so we can connect directly to it
		# First, make sure it has a collision shape
		var existing_collision = water_volume.get_node("CollisionShape")
		if existing_collision:
			water_volume.body_entered.connect(_on_body_entered_water)
			water_volume.body_exited.connect(_on_body_exited_water)
		else:
			print("Warning: WaterVolume Area3D has no CollisionShape3D child")

func _on_body_entered_water(body: Node):
	if body.has_method("enter_water"):
		body.call("enter_water")
	print("Object entered water: " + body.name)

func _on_body_exited_water(body: Node):
	if body.has_method("exit_water"):
		body.call("exit_water")
	print("Object exited water: " + body.name)

func add_object(object_scene: PackedScene, position: Vector3, rotation: Vector3 = Vector3.ZERO, scale: Vector3 = Vector3.ONE) -> Node3D:
	var object_instance = object_scene.instantiate()
	add_child(object_instance)
	object_instance.translation = position
	object_instance.rotation = rotation
	object_instance.scale = scale
	objects.append(object_instance)
	return object_instance

func add_target(target_scene: PackedScene, position: Vector3, rotation: Vector3 = Vector3.ZERO) -> Node3D:
	var target_instance = target_scene.instantiate()
	add_child(target_instance)
	target_instance.translation = position
	target_instance.rotation = rotation
	targets.append(target_instance)
	return target_instance

func get_water_level() -> float:
	return water_surface.global_transform.origin.y

func get_closest_target(position: Vector3) -> Node3D:
	if targets.is_empty():
		return null
	var closest = targets[0]
	var closest_distance = position.distance_to(closest.global_transform.origin)
	for target in targets:
		var distance = position.distance_to(target.global_transform.origin)
		if distance < closest_distance:
			closest = target
			closest_distance = distance
	return closest
