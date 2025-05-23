extends Camera

# Depth camera properties
export var max_depth := 10.0  # Maximum depth in meters
export var min_depth := 0.1   # Minimum depth in meters
export var noise_amount := 0.02  # Noise in depth measurements

# Output resolution
export var depth_width := 320
export var depth_height := 240

# Internal variables
var depth_texture: ImageTexture
var depth_image: Image

func _ready():
    # Initialize depth image
    depth_image = Image.new()
    depth_image.create(depth_width, depth_height, false, Image.FORMAT_RF)
    
    depth_texture = ImageTexture.new()
    depth_texture.create_from_image(depth_image)
    
    print("Depth camera initialized")

func update_depth_image():
    # This would normally be done with a shader, but this is a simplified version
    # In an actual implementation, you'd use a depth pre-pass render
    
    # Get the current viewport (needed for raycasting)
    var space_state = get_world().direct_space_state
    
    # Calculate step size
    var x_step = 1.0 / depth_width
    var y_step = 1.0 / depth_height
    
    # Iterate through the image grid
    for y in range(depth_height):
        for x in range(depth_width):
            # Calculate normalized device coordinates (-1 to 1)
            var ndc_x = (x / float(depth_width)) * 2.0 - 1.0
            var ndc_y = (y / float(depth_height)) * 2.0 - 1.0
            
            # Cast ray from camera
            var from = global_transform.origin
            var to = project_position(Vector2(ndc_x, ndc_y), max_depth)
            
            # Perform raycast
            var result = space_state.intersect_ray(from, to)
            
            var depth_value: float
            if result and result.has("position"):
                # Calculate distance from camera to hit point
                depth_value = from.distance_to(result.position)
                
                # Apply some noise to simulate sensor noise
                depth_value += rand_range(-noise_amount, noise_amount)
                
                # Clamp to valid range
                depth_value = clamp(depth_value, min_depth, max_depth)
            else:
                # No hit, use max depth
                depth_value = max_depth
            
            # Normalize depth to 0-1 range for visualization
            var normalized_depth = depth_value / max_depth
            
            # Set pixel in depth image
            depth_image.lock()
            depth_image.set_pixel(x, y, Color(depth_value, 0, 0))
            depth_image.unlock()
    
    # Update texture
    depth_texture.create_from_image(depth_image)

func get_depth_image() -> Image:
    return depth_image.duplicate()

func get_depth_texture() -> ImageTexture:
    return depth_texture

func get_depth_array() -> PoolRealArray:
    # Creates a flat array of depth values (for ROS)
    var array = PoolRealArray()
    array.resize(depth_width * depth_height)
    
    depth_image.lock()
    
    for y in range(depth_height):
        for x in range(depth_width):
            var color = depth_image.get_pixel(x, y)
            array[y * depth_width + x] = color.r  # Depth value stored in red channel
    
    depth_image.unlock()
    
    return array

func _process(_delta):
    # Update depth image periodically (could be set to lower frequency)
    update_depth_image()