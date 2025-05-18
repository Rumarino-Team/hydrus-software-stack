extends Spatial

# Target properties
export var target_type := "buoy"  # Type identifier (buoy, gate, bin, etc)
export var target_color := Color(1.0, 0.0, 0.0, 1.0)  # Default: red
export var target_scale := 1.0
export var target_id := 0  # Unique ID
export var is_moving := false
export var movement_speed := 0.1
export var movement_range := Vector3(1, 0.5, 1)

# Movement variables
var initial_position := Vector3.ZERO
var movement_time := 0.0

# Visibility from cameras
onready var mesh_instance = $MeshInstance
onready var collision_shape = $CollisionShape

func _ready():
    # Store initial position for movement
    initial_position = global_transform.origin
    
    # Set up visual appearance
    if mesh_instance:
        var material = SpatialMaterial.new()
        material.albedo_color = target_color
        material.metallic = 0.2
        material.roughness = 0.7
        material.emission_enabled = true
        material.emission = target_color
        material.emission_energy = 0.2
        mesh_instance.set_surface_material(0, material)
        
        # Apply scale
        mesh_instance.scale = Vector3(target_scale, target_scale, target_scale)
        
        if collision_shape:
            collision_shape.scale = mesh_instance.scale

func _process(delta):
    if is_moving:
        # Simple oscillating movement
        movement_time += delta
        
        var x_offset = sin(movement_time * 0.5) * movement_range.x
        var y_offset = sin(movement_time * 0.7) * movement_range.y
        var z_offset = cos(movement_time * 0.3) * movement_range.z
        
        global_transform.origin = initial_position + Vector3(x_offset, y_offset, z_offset)

func get_detection_info():
    # Return information about this target for detection systems
    return {
        "type": target_type,
        "color": {
            "r": target_color.r,
            "g": target_color.g,
            "b": target_color.b
        },
        "position": {
            "x": global_transform.origin.x,
            "y": global_transform.origin.y,
            "z": global_transform.origin.z
        },
        "scale": target_scale,
        "id": target_id
    }

func highlight():
    # Visual feedback when detected
    if mesh_instance:
        var material = mesh_instance.get_surface_material(0)
        if material:
            material.emission_energy = 1.0
            
            # Create a tween to reset emission
            var tween = Tween.new()
            add_child(tween)
            tween.interpolate_property(material, "emission_energy", 
                                      1.0, 0.2, 0.5, 
                                      Tween.TRANS_QUAD, Tween.EASE_OUT)
            tween.start()
            
            yield(tween, "tween_completed")
            tween.queue_free()

func _on_Area_body_entered(body):
    # Called when a body enters the detection area
    if body.has_method("target_detected"):
        body.target_detected(self)
        highlight()