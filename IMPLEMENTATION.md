# Runtime Parameter Reconfiguration Implementation

## Overview

This document describes the implementation of runtime parameter reconfiguration for the `robot_self_filter` package. This feature allows users to dynamically adjust filter parameters (especially scale and padding values for collision shapes) without restarting the ROS 2 node.

## Problem Statement

Previously, the robot self-filter package read all parameters once during initialization. If users wanted to tune the filter parameters (such as increasing the scale or padding of specific robot links), they had to:
1. Stop the node
2. Edit the YAML configuration file
3. Restart the node
4. Repeat until satisfied

This made the tuning process slow and tedious, especially when fine-tuning multiple links or testing different values in real-time.

## Solution

We implemented a runtime parameter reconfiguration system using ROS 2's parameter callback mechanism. The solution consists of:

1. **Parameter Callback Handler** - Monitors parameter changes and applies them in real-time
2. **Dynamic Update Methods** - Updates the internal collision body representations without full reinitialization
3. **Shape Type Awareness** - Handles different shape types (sphere, box, cylinder) with appropriate parameter dimensions

## Technical Implementation

### 1. Added Parameter Callback in `self_see_filter.h`

**Key Changes:**
- Added `param_callback_handle_` member variable to store the callback handle
- Added `link_names_` vector to track configured links
- Registered callback using `node_->add_on_set_parameters_callback()`

```cpp
// In constructor
param_callback_handle_ = node_->add_on_set_parameters_callback(
  std::bind(&SelfFilter::parametersCallback, this, std::placeholders::_1));
```

### 2. Implemented Parameter Callback Method

The `parametersCallback` method handles all parameter changes:

**Supported Parameters:**
- **General filtering parameters:**
  - `min_sensor_dist` - Minimum distance from sensor
  - `keep_organized` - Maintain organized point cloud structure
  - `zero_for_removed_points` - Set filtered points to zero vs NaN
  - `invert` - Invert filter behavior

- **Default shape parameters:**
  - `default_sphere_scale` and `default_sphere_padding`
  - `default_box_scale` and `default_box_padding` (3D arrays)
  - `default_cylinder_scale` and `default_cylinder_padding` (2D arrays)

- **Per-link parameters:**
  - `self_see_links.<link_name>.scale` - Single value for spheres
  - `self_see_links.<link_name>.padding` - Single value for spheres
  - `self_see_links.<link_name>.box_scale` - 3D array [x, y, z]
  - `self_see_links.<link_name>.box_padding` - 3D array [x, y, z]
  - `self_see_links.<link_name>.cylinder_scale` - 2D array [radial, vertical]
  - `self_see_links.<link_name>.cylinder_padding` - 2D array [radial, vertical]

### 3. Added Update Methods to `self_mask.h`

**New Enums:**
```cpp
enum class ScaleType {
  SPHERE,
  BOX,
  CYLINDER
};
```

**New Methods:**
- `updateLinkScale(link_name, values, type)` - Updates scale for a specific link
- `updateLinkPadding(link_name, values, type)` - Updates padding for a specific link

**Implementation Details:**
1. Finds the body by link name
2. Casts to the appropriate body type (Sphere, Box, or Cylinder)
3. Applies the new scale/padding values
4. Recomputes the body volume
5. Re-sorts bodies by volume (optimization for filtering)

### 4. Shape-Specific Handling

Each shape type has different dimensionality for scale and padding:

- **Sphere**: Single value (uniform scaling)
- **Box**: 3 values (x, y, z dimensions)
- **Cylinder**: 2 values (radial and vertical)

The implementation validates that the correct number of values is provided for each shape type.

## Usage Examples

### Command Line Usage

```bash
# Update sphere scale (single value)
ros2 param set /self_filter self_see_links.BOOM.scale 1.2

# Update box scale (3 values)
ros2 param set /self_filter self_see_links.STICK.box_scale "[1.1, 1.1, 1.7]"

# Update cylinder padding (2 values)
ros2 param set /self_filter self_see_links.LF_WHEEL.cylinder_padding "[0.05, 0.3]"

# Update general parameters
ros2 param set /self_filter min_sensor_dist 2.5

# List all parameters
ros2 param list /self_filter

# Get current value
ros2 param get /self_filter self_see_links.BOOM.scale
```

### Interactive Script

An interactive helper script is provided:

```bash
./scripts/runtime_param_example.sh
```

This script offers a menu-driven interface for:
1. Listing all parameters
2. Viewing link-specific parameters
3. Adjusting scale values
4. Adjusting padding values
5. Viewing usage examples

## Benefits

1. **Faster Tuning** - Adjust parameters in real-time without node restarts
2. **Interactive Workflow** - See immediate feedback in RViz and filtered point clouds
3. **Experimentation** - Easily test different values to find optimal settings
4. **Debugging** - Quickly adjust parameters to diagnose filtering issues
5. **Production Use** - Adapt to changing conditions or robot configurations on-the-fly

## Backward Compatibility

The implementation is fully backward compatible:
- All existing YAML configurations work without changes
- Parameters are still read from YAML at startup
- Runtime changes are optional - nodes work as before if not used
- No changes to launch files or command-line arguments required

## Performance Considerations

- Parameter updates are efficient (O(n) where n = number of bodies for that link)
- Volume recomputation happens only for updated bodies
- Bodies are re-sorted after updates to maintain filtering optimization
- No memory allocation/deallocation during updates

## Testing Recommendations

Since we don't have a full ROS 2 environment in the current setup, we recommend the following tests in a ROS 2 environment:

1. **Basic Parameter Change Test**
   - Start node with default parameters
   - Change a link scale parameter
   - Verify the collision shape marker updates in RViz
   - Verify filtered point cloud changes accordingly

2. **Multiple Links Test**
   - Change parameters for multiple links
   - Verify all changes are applied correctly

3. **Shape Type Test**
   - Test sphere parameters (single values)
   - Test box parameters (3D arrays)
   - Test cylinder parameters (2D arrays)

4. **Edge Cases**
   - Invalid link names (should be ignored)
   - Wrong array sizes (should be validated)
   - Extreme values (verify robustness)

5. **Performance Test**
   - Measure update latency
   - Test with many links configured
   - Monitor CPU usage during parameter changes

## Files Modified

1. **include/robot_self_filter/self_see_filter.h**
   - Added parameter callback infrastructure
   - Added parametersCallback method
   - Added link_names_ tracking

2. **include/robot_self_filter/self_mask.h**
   - Added ScaleType enum
   - Added updateLinkScale method
   - Added updateLinkPadding method

3. **README.md**
   - Added "Runtime Parameter Changes" section
   - Updated troubleshooting section
   - Added usage examples

4. **scripts/runtime_param_example.sh**
   - New interactive helper script

5. **CMakeLists.txt**
   - Updated to install scripts directory

6. **.gitignore**
   - Added to exclude build artifacts

## Future Enhancements

Possible future improvements:
1. Parameter validation with min/max ranges
2. Parameter presets for common scenarios
3. Automatic parameter saving to YAML
4. Dynamic service for batch parameter updates
5. Parameter change history/undo functionality

## Conclusion

The runtime parameter reconfiguration feature significantly improves the usability of the robot_self_filter package by enabling dynamic tuning. The implementation is clean, efficient, and maintains full backward compatibility with existing configurations.
