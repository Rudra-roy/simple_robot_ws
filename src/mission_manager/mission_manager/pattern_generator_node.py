#!/usr/bin/env python3
"""
Pattern Generator Node
Generates geometric patterns (square, circle) around a GPS center point.
"""

import rclpy
from rclpy.node import Node
from rclpy.service import Service
import math


class PatternGeneratorNode(Node):
    """Generates geometric waypoint patterns."""
    
    def __init__(self):
        super().__init__('pattern_generator_node')
        
        # Earth radius for GPS calculations
        self.EARTH_RADIUS = 6371000.0  # meters
        
        self.get_logger().info('Pattern Generator Node initialized')
    
    def meters_to_lat(self, meters_north):
        """Convert meters north to latitude degrees."""
        return meters_north / self.EARTH_RADIUS * (180.0 / math.pi)
    
    def meters_to_lon(self, meters_east, center_lat):
        """Convert meters east to longitude degrees at given latitude."""
        return meters_east / (self.EARTH_RADIUS * math.cos(math.radians(center_lat))) * (180.0 / math.pi)
    
    def generate_square_pattern(self, center_lat, center_lon, size_meters):
        """Generate square pattern waypoints."""
        half_size = size_meters / 2.0
        
        # Calculate offsets
        lat_offset = self.meters_to_lat(half_size)
        lon_offset = self.meters_to_lon(half_size, center_lat)
        
        # Generate 4 corners (clockwise from NE)
        waypoints = [
            {'lat': center_lat + lat_offset, 'lon': center_lon + lon_offset, 'task_type': None},  # NE
            {'lat': center_lat - lat_offset, 'lon': center_lon + lon_offset, 'task_type': None},  # SE
            {'lat': center_lat - lat_offset, 'lon': center_lon - lon_offset, 'task_type': None},  # SW
            {'lat': center_lat + lat_offset, 'lon': center_lon - lon_offset, 'task_type': None},  # NW
        ]
        
        return waypoints
    
    def generate_circle_pattern(self, center_lat, center_lon, radius_meters, num_points=8):
        """Generate circular pattern waypoints."""
        waypoints = []
        
        for i in range(num_points):
            angle = 2.0 * math.pi * i / num_points  # Radians
            
            # Calculate offset in meters
            north = radius_meters * math.cos(angle)
            east = radius_meters * math.sin(angle)
            
            # Convert to lat/lon
            lat = center_lat + self.meters_to_lat(north)
            lon = center_lon + self.meters_to_lon(east, center_lat)
            
            waypoints.append({'lat': lat, 'lon': lon, 'task_type': None})
        
        return waypoints
    
    def save_pattern_to_yaml(self, waypoints, output_file):
        """Save pattern waypoints to YAML file."""
        import yaml
        
        mission_data = {
            'mission_name': 'Generated Pattern',
            'waypoints': waypoints
        }
        
        with open(output_file, 'w') as f:
            yaml.dump(mission_data, f, default_flow_style=False)
        
        self.get_logger().info(f'Pattern saved to {output_file}')


def main(args=None):
    rclpy.init(args=args)
    node = PatternGeneratorNode()
    
    # Example: Generate square pattern around current position
    # This would normally be called via a service or parameters
    center_lat = 40.712776  # Example
    center_lon = -74.005974  # Example
    
    square_waypoints = node.generate_square_pattern(center_lat, center_lon, 4.0)
    node.get_logger().info(f'Generated square pattern: {square_waypoints}')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
