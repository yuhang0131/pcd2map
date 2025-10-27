# Pointcloud to Costmap Converter

A ROS2 package for converting 3D point cloud data (PCD files) into 2D occupancy grid maps (costmaps) with advanced filtering capabilities for noise reduction and obstacle detection.

## Overview

This package provides a robust solution for generating navigation-ready costmaps from 3D point cloud data. It features a multi-stage filtering pipeline that effectively removes noise, outliers, and unwanted points while preserving important obstacles and structures.

## Features

- **Multi-stage Filtering Pipeline**: Height filtering, statistical outlier removal, radius filtering, normal vector filtering, and voxel grid filtering
- **High Noise Reduction**: Achieves 90-98% noise reduction while preserving important obstacles
- **Configurable Parameters**: Comprehensive parameter system for fine-tuning filtering behavior
- **Multiple Output Formats**: Generates both PGM image files and YAML configuration files for ROS navigation
- **Debug Support**: Optional intermediate result saving and verbose logging

## Installation

### Prerequisites

- ROS2 Humble or later
- PCL (Point Cloud Library) 1.12+
- OpenCV 4.x
- CMake 3.8+

### Build Instructions

```bash
# Navigate to your ROS2 workspace
cd /path/to/your_ws

# Clone or copy the package to src/
# src/pointcloud_to_costmap/

# Build the package
colcon build --packages-select pointcloud_to_costmap

# Source the workspace
source install/setup.bash
```

## Usage

### Quick Start

1. **Configure parameters**: Edit `param/param.yaml` to set your input PCD file path and filtering parameters
2. **Launch the converter**: 
   ```bash
   ros2 launch pointcloud_to_costmap pointcloud_to_costmap.launch.py
   ```

### Parameter Configuration

Edit the `param/param.yaml` file to configure the converter for your specific use case:

```yaml
pointcloud_to_costmap_node:
  ros__parameters:
    # Input/Output Configuration
    input_pcd: "/path/to/your/pointcloud.pcd"
    output_prefix: "your_map_name"
    output_directory: "/path/to/output/"
    
    # Filtering parameters...
```

### Command Line Usage

```bash
# Basic usage
ros2 launch pointcloud_to_costmap pointcloud_to_costmap.launch.py

# With custom parameter file
ros2 launch pointcloud_to_costmap pointcloud_to_costmap.launch.py param_file:=/path/to/custom_params.yaml
```

## Parameters

### Input/Output Configuration

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `input_pcd` | string | "map.pcd" | Path to input PCD file |
| `output_prefix` | string | "map" | Prefix for output files |
| `output_directory` | string | "/tmp/costmap_output" | Directory for saving output files |

### Map Generation Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `resolution` | double | 0.05 | Map resolution in meters per pixel |
| `padding` | int | 10 | Border padding in pixels |

### Height Filtering Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_height` | double | -0.5 | Minimum height threshold (meters) |
| `max_height` | double | 3.0 | Maximum height threshold (meters) |

### Normal Vector Filtering Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `normal_radius` | double | 0.3 | Radius for normal estimation (meters) |
| `wall_normal_threshold` | double | 0.3 | Threshold for wall normal detection |

### Statistical Filtering Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `statistical_k` | int | 20 | Number of neighbors for statistical analysis |
| `statistical_stddev` | double | 1.0 | Standard deviation multiplier for outlier detection |

### Radius Filtering Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `radius_outlier_radius` | double | 0.2 | Search radius for neighbor counting (meters) |
| `radius_outlier_min_neighbors` | int | 10 | Minimum number of neighbors required |

### Processing Options

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_height_filter` | bool | true | Enable/disable height filtering |
| `enable_statistical_filter` | bool | true | Enable/disable statistical outlier removal |
| `enable_radius_filter` | bool | true | Enable/disable radius outlier removal |
| `enable_normal_filter` | bool | true | Enable/disable normal vector filtering |

### Debug Options

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `save_intermediate_clouds` | bool | false | Save intermediate filtering results |
| `verbose_logging` | bool | true | Enable detailed logging output |

## Filtering Pipeline

The package uses a multi-stage filtering approach:

1. **Height Filtering**: Removes points outside the specified height range (ground/ceiling removal)
2. **Statistical Filtering**: Removes statistical outliers based on neighbor analysis
3. **Radius Filtering**: Removes isolated points with insufficient neighbors
4. **Normal Vector Filtering**: Removes wall points based on surface normal analysis
5. **Voxel Grid Filtering**: Downsamples the point cloud for uniform density

## Output Files

The converter generates two output files:

- **MAP_NAME.pgm**: Grayscale occupancy grid image
- **MAP_NAME.yaml**: ROS-compatible map configuration file

### YAML Configuration Format

```yaml
image: MAP_NAME.pgm
resolution: 0.05
origin: [-14.0075, -80.5393, 0.0]
negate: 0
occupied_thresh: 0.1
free_thresh: 0.9
```

## Performance

Typical noise reduction performance:

- **Indoor environments**: 90-95% noise reduction
- **Outdoor environments**: 95-98% noise reduction
- **Processing time**: 2-10 seconds depending on point cloud size

## Example Results

### Indoor Environment
- Input: 397,040 points
- Output: 36,361 points (90.8% reduction)
- Processing time: ~2 seconds

### Outdoor Environment
- Input: 2,545,620 points  
- Output: 31,058 points (98.8% reduction)
- Processing time: ~8 seconds

## Troubleshooting

### Common Issues

1. **File not found error**: Check that the `input_pcd` path is correct and the file exists
2. **Permission denied**: Ensure the output directory is writable
3. **Empty output**: Adjust filtering parameters if too aggressive

### Parameter Tuning Tips

- **Too much noise**: Decrease `statistical_stddev`, increase `radius_outlier_min_neighbors`
- **Missing obstacles**: Increase `statistical_stddev`, decrease `radius_outlier_min_neighbors`
- **Slow processing**: Increase `resolution`, decrease `statistical_k`

## Dependencies

```xml
<depend>rclcpp</depend>
<depend>pcl_ros</depend>
<depend>pcl_conversions</depend>
<depend>opencv</depend>
<depend>libpcl-all-dev</depend>
```

## License

This package is released under the MIT License.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bug reports and feature requests.

## Authors

- Developer: [Your Name]
- Maintainer: [Your Email]

## Changelog

### Version 1.0.0
- Initial release with multi-stage filtering pipeline
- Parameter-based configuration system
- Launch file support
- Comprehensive documentation
