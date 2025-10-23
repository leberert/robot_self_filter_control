# robot_self_filter

A ROS 2 package that filters robot body parts from point cloud data, enabling sensors to ignore the robot's own geometry during perception tasks.

## Overview

The `robot_self_filter` package removes points from sensor data that correspond to the robot's body, preventing self-occlusion issues in perception pipelines. It uses the robot's URDF model to create collision shapes and filters out points that fall within these shapes.

### Key Features

- **Multi-sensor support**: Compatible with various LiDAR sensors (Ouster, Hesai, Robosense, Pandar)
- **Dynamic filtering**: Updates collision shapes based on robot joint states
- **Configurable shapes**: Supports spheres, boxes, and cylinders with customizable padding and scaling
- **Visualization**: Publishes collision shapes for debugging and tuning
- **Performance optimized**: Efficient point cloud processing using PCL

## Installation

### Prerequisites

- ROS 2 (tested on Humble/Iron)
- Dependencies:
  - `rclcpp`
  - `tf2_ros`
  - `sensor_msgs`
  - `geometry_msgs`
  - `visualization_msgs`
  - `urdf`
  - `resource_retriever`
  - `pcl_ros`
  - `filters`
  - `bullet`
  - `assimp`

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select robot_self_filter
source install/setup.bash
```

## Usage

### Quick Start

Launch the self filter node with your robot configuration:

```bash
ros2 launch robot_self_filter self_filter.launch.py \
    robot_description:="$(xacro /path/to/robot.urdf.xacro)" \
    filter_config:=/path/to/filter_config.yaml \
    in_pointcloud_topic:=/lidar/points \
    out_pointcloud_topic:=/lidar/points_filtered
```

### Launch Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `robot_description` | string | - | Robot URDF/XACRO description |
| `filter_config` | string | - | Path to YAML configuration file |
| `in_pointcloud_topic` | string | `/cloud_in` | Input point cloud topic |
| `out_pointcloud_topic` | string | `/cloud_out` | Filtered point cloud topic |
| `lidar_sensor_type` | int | `2` | Sensor type (0: XYZ, 1: XYZRGB, 2: Ouster, 3: Hesai, 4: Robosense, 5: Pandar) |
| `zero_for_removed_points` | bool | `true` | Set filtered points to zero instead of removing |
| `use_sim_time` | bool | `true` | Use simulation time |
| `description_name` | string | `/robot_description` | Robot description parameter namespace |

## Configuration

### YAML Configuration File

The filter configuration is defined in a YAML file. See `params/example.yaml` for a complete example.

#### Basic Structure

```yaml
self_filter_node:
  ros__parameters:
    sensor_frame: "lidar_frame"  # Frame of the sensor
    
    # Default shape parameters
    default_sphere_scale: 1.0
    default_sphere_padding: 0.01
    
    default_box_scale: [1.0, 1.0, 1.0]  # [x, y, z]
    default_box_padding: [0.01, 0.01, 0.01]  # [x, y, z]
    
    default_cylinder_scale: [1.0, 1.0]  # [radial, vertical]
    default_cylinder_padding: [0.01, 0.01]  # [radial, vertical]
    
    # Filtering parameters
    keep_organized: false  # Maintain organized point cloud structure
    zero_for_removed_points: false  # Zero points instead of removing
    invert: false  # Invert filter (keep only robot points)
    min_sensor_dist: 0.5  # Minimum distance from sensor to filter
    
    # Links to filter
    self_see_links:
      names:
        - base_link
        - arm_link_1
        - arm_link_2
        # ... add all robot links to filter
      
      # Per-link custom parameters (optional)
      arm_link_1:
        box_scale: [1.1, 1.1, 1.0]
        box_padding: [0.05, 0.05, 0.02]
```

### Shape Types

The filter automatically determines shape types from the robot's URDF collision geometry:

- **Sphere**: Single scale and padding value
- **Box**: 3D scale and padding values [x, y, z]
- **Cylinder**: 2D scale and padding [radial, vertical]

### Tuning Guidelines

1. **Start with default values**: Use scale=1.0 and small padding (0.01-0.05)
2. **Visualize collision shapes**: Check `/collision_shapes` topic in RViz
3. **Adjust per-link parameters**: Fine-tune problematic links individually
4. **Consider sensor noise**: Increase padding for noisy sensors
5. **Monitor performance**: Larger padding values may filter valid points

## Topics

### Subscribed Topics

- `<in_pointcloud_topic>` (sensor_msgs/PointCloud2): Raw point cloud from sensor
- `/tf` (tf2_msgs/TFMessage): Transform data
- `/tf_static` (tf2_msgs/TFMessage): Static transforms
- `/joint_states` (sensor_msgs/JointState): Robot joint positions

### Published Topics

- `<out_pointcloud_topic>` (sensor_msgs/PointCloud2): Filtered point cloud
- `/collision_shapes` (visualization_msgs/MarkerArray): Visualization of filter shapes

## Visualization

To visualize the collision shapes in RViz:

1. Add a MarkerArray display
2. Set topic to `/collision_shapes`
3. Collision shapes will appear as semi-transparent geometries

## Sensor Types

The package supports multiple sensor types through the `lidar_sensor_type` parameter:

| Value | Sensor Type | Point Type |
|-------|-------------|------------|
| 0 | Generic XYZ | `pcl::PointXYZ` |
| 1 | Generic XYZRGB | `pcl::PointXYZRGB` |
| 2 | Ouster | Custom Ouster point type |
| 3 | Hesai | Custom Hesai point type |
| 4 | Robosense | Custom Robosense point type |
| 5 | Pandar | Custom Pandar point type |

## Examples

### Example 1: Mobile Robot with Arm

```yaml
self_filter_mobile_robot:
  ros__parameters:
    sensor_frame: "lidar_link"
    min_sensor_dist: 0.3
    
    self_see_links:
      names:
        - base_link
        - wheel_left
        - wheel_right
        - arm_base
        - arm_link_1
        - arm_link_2
        - gripper
      
      # Wheels need extra padding for suspension movement
      wheel_left:
        cylinder_scale: [1.0, 1.0]
        cylinder_padding: [0.05, 0.1]
      
      wheel_right:
        cylinder_scale: [1.0, 1.0]
        cylinder_padding: [0.05, 0.1]
```

### Example 2: Construction Equipment

See `params/example.yaml` for a complete configuration example for construction equipment with multiple moving parts.

## Runtime Parameter Changes

The package supports **dynamic parameter reconfiguration** at runtime, allowing you to adjust filtering parameters without restarting the node. This is especially useful for tuning scale and padding values for different robot links.

### Changing Parameters at Runtime

Use the `ros2 param set` command to update parameters while the node is running:

```bash
# Update scale for a specific link (for sphere/cylinder)
ros2 param set /self_filter self_see_links.BOOM.scale 1.2

# Update padding for a specific link
ros2 param set /self_filter self_see_links.BOOM.padding 0.15

# Update box scale (3 values: x, y, z)
ros2 param set /self_filter self_see_links.STICK.box_scale "[1.1, 1.1, 1.7]"

# Update box padding (3 values: x, y, z)
ros2 param set /self_filter self_see_links.STICK.box_padding "[0.05, 0.22, 0.05]"

# Update cylinder scale (2 values: radial, vertical)
ros2 param set /self_filter self_see_links.LF_WHEEL.cylinder_scale "[1.0, 1.0]"

# Update cylinder padding (2 values: radial, vertical)
ros2 param set /self_filter self_see_links.LF_WHEEL.cylinder_padding "[0.05, 0.3]"

# Update default values
ros2 param set /self_filter default_sphere_scale 1.1
ros2 param set /self_filter default_sphere_padding 0.02
ros2 param set /self_filter default_box_scale "[1.0, 1.0, 1.0]"
ros2 param set /self_filter default_box_padding "[0.01, 0.01, 0.01]"

# Update filtering behavior parameters
ros2 param set /self_filter min_sensor_dist 2.5
ros2 param set /self_filter keep_organized false
ros2 param set /self_filter zero_for_removed_points false
```

### List Current Parameters

View all current parameter values:

```bash
ros2 param list /self_filter
ros2 param get /self_filter self_see_links.BOOM.scale
```

### Parameter Types by Shape

- **Sphere**: `scale` (single value), `padding` (single value)
- **Box**: `box_scale` (3 values: x,y,z), `box_padding` (3 values: x,y,z)
- **Cylinder**: `cylinder_scale` (2 values: radial, vertical), `cylinder_padding` (2 values: radial, vertical)

### Runtime Tuning Workflow

1. **Start the node** with initial parameter values from YAML
2. **Visualize collision shapes** in RViz using `/collision_shapes` topic
3. **Identify problematic links** that need adjustment
4. **Update parameters dynamically** using `ros2 param set`
5. **Observe changes** in real-time in RViz and filtered point cloud
6. **Save working values** back to YAML file for future use

### Interactive Helper Script

An interactive helper script is provided for convenience:

```bash
# Run the interactive parameter adjustment script
./scripts/runtime_param_example.sh

# Or specify a different node name
./scripts/runtime_param_example.sh /my_custom_filter_node
```

The script provides an interactive menu for:
- Listing all parameters
- Viewing link-specific parameters
- Adjusting scale values
- Adjusting padding values
- Viewing usage examples

### Example: Tuning a Robotic Arm

```bash
# If arm links are filtering too aggressively
ros2 param set /self_filter self_see_links.arm_link_1.box_scale "[0.9, 0.9, 0.9]"
ros2 param set /self_filter self_see_links.arm_link_1.box_padding "[0.01, 0.01, 0.01]"

# If base is not filtering enough
ros2 param set /self_filter self_see_links.base_link.scale 1.2
ros2 param set /self_filter self_see_links.base_link.padding 0.10
```

## Troubleshooting

### Common Issues

1. **Points not being filtered**
   - Check sensor frame matches configuration
   - Verify TF tree is complete
   - Increase padding values (try runtime adjustment first)
   - Check URDF collision geometries

2. **Too many points filtered**
   - Reduce padding values using runtime parameter changes
   - Check scale factors (should typically be 1.0)
   - Verify min_sensor_dist setting

3. **Performance issues**
   - Reduce number of filtered links
   - Simplify collision geometries in URDF
   - Consider using `zero_for_removed_points` for organized clouds

4. **No output point cloud**
   - Verify input topic is publishing
   - Check remappings in launch file
   - Ensure robot_description is loaded

5. **Runtime parameter changes not working**
   - Verify the link name matches exactly (case-sensitive)
   - Check parameter type matches shape type (box, cylinder, sphere)
   - Use `ros2 param list` to see available parameters

## Contributing

Contributions are welcome! Please ensure your code follows ROS 2 coding standards and includes appropriate documentation.

## License

BSD License - See package.xml for details

## Maintainer

Lorenzo Terenzi <lterenzi@ethz.ch>

## Author

ROS 2 version by Lorenzo Terenzi
Original ROS 1 version by Eitan Marder-Eppstein