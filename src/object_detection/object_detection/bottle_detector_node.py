#!/usr/bin/env python3
"""
Bottle Detector Node
Detects and tracks bottle objects using YOLO.
"""

import rclpy
from object_detection.yolo_detector_base import YOLODetectorBase


class BottleDetectorNode(YOLODetectorBase):
    """Bottle-specific detector node."""
    
    def __init__(self):
        super().__init__('bottle_detector_node', 'bottle')


def main(args=None):
    rclpy.init(args=args)
    node = BottleDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
