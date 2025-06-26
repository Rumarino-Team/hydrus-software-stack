extends Spatial

# Water environment properties
export var water_color := Color(0.1, 0.4, 0.8, 0.8)
export var water_depth := 10.0
export var water_clarity := 0.5  # 0-1, affects visibility distance

# Lighting properties
export var ambient_light_color := Color(0.05, 0.15, 0.3, 1.0)
export var directional_light_color := Color(0.8, 0.8, 1.0, 1.0)
export var directional_light_energy := 0.6

# References to scene nodes
onready var water_surface = $WaterSurface
onready var water_volume = $WaterVolume
onready var directional_light = $DirectionalLight
onready var ambient_light = $AmbientLight

# Objects in the environment
var objects = []
var targets = []

func _ready():
    # Initialize water properties
    if water_surface:
        water_surface.get_surface_material(0).albedo_color = water_color

    if water_volume:
        var volume_material = water_volume.get_surface_material(0)
        if volume_material:
            volume_material.albedo_color = Color(water_color.r, water_color.g, water_color.b, 0.1)

    # Initialize lighting
    if ambient_light:
        ambient_light.light_color = ambient_light_color

    if directional_light:
        directional_light.light_color = directional_light_color
        directional_light.light_energy = directional_light_energy

    # Apply water effects to cameras in the scene
    setup_underwater_effects()

    # Add collision detection for water volume
    add_water_collision()

    print("Water environment initialized")

func setup_underwater_effects():
    # Find all cameras in the scene
    var cameras = find_cameras(self)

    for camera in cameras:
        # Apply underwater post-processing effect
        var environment = camera.get_environment()
        if not environment:
            environment = Environment.new()
            camera.set_environment(environment)

        # Set underwater fog
        environment.fog_enabled = true
        environment.fog_color = water_color
        environment.fog_depth_begin = 1.0
        environment.fog_depth_end = 15.0 * water_clarity

        # Underwater light absorption (color correction)
        environment.adjustment_enabled = true
        environment.adjustment_brightness = 0.9
        environment.adjustment_contrast = 1.1
        environment.adjustment_saturation = 1.2

        # Add slight color shifting toward blue/green
        var color_correction = environment.get_adjustment_color_correction()
        if not color_correction:
            # Create a default blue-green tinted correction
            var gradient = Gradient.new()
            gradient.add_point(0.0, Color(0.8, 0.9, 1.0))
            gradient.add_point(1.0, Color(1.0, 1.0, 1.0))

            var gradient_texture = GradientTexture.new()
            gradient_texture.gradient = gradient

            environment.adjustment_color_correction = gradient_texture

func find_cameras(node):
    var cameras = []

    if node is Camera:
        cameras.append(node)

    for child in node.get_children():
        cameras += find_cameras(child)

    return cameras

func add_water_collision():
    # Add area node for water effects
    if water_volume:
        var water_area = Area.new()
        water_area.name = "WaterArea"
        water_volume.add_child(water_area)

        var collision_shape = CollisionShape.new()
        collision_shape.shape = water_volume.mesh.create_convex_shape()
        water_area.add_child(collision_shape)

        # Connect signals
        water_area.connect("body_entered", self, "_on_body_entered_water")
        water_area.connect("body_exited", self, "_on_body_exited_water")

func _on_body_entered_water(body):
    # Apply water effects to objects entering water
    if body.has_method("enter_water"):
        body.enter_water()

    # Notify via print for debugging
    print("Object entered water: " + body.name)

func _on_body_exited_water(body):
    # Remove water effects from objects exiting water
    if body.has_method("exit_water"):
        body.exit_water()

    # Notify via print for debugging
    print("Object exited water: " + body.name)

func add_object(object_scene, position, rotation=Vector3(0,0,0), scale=Vector3(1,1,1)):
    # Instance the object
    var object_instance = object_scene.instance()
    add_child(object_instance)

    # Set transform
    object_instance.translation = position
    object_instance.rotation = rotation
    object_instance.scale = scale

    # Add to objects list
    objects.append(object_instance)

    return object_instance

func add_target(target_scene, position, rotation=Vector3(0,0,0)):
    # Instance the target
    var target_instance = target_scene.instance()
    add_child(target_instance)

    # Set transform
    target_instance.translation = position
    target_instance.rotation = rotation

    # Add to targets list
    targets.append(target_instance)

    return target_instance

func get_water_level():
    # Return the current water level (y-coordinate)
    return water_surface.global_transform.origin.y

func get_closest_target(position):
    if targets.empty():
        return null

    var closest = targets[0]
    var closest_distance = position.distance_to(closest.global_transform.origin)

    for target in targets:
        var distance = position.distance_to(target.global_transform.origin)
        if distance < closest_distance:
            closest = target
            closest_distance = distance

    return closest
