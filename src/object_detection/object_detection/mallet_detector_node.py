#!/usr/bin/env python3
"""
Mallet Detector Node
Detects and tracks mallet objects using YOLO.
"""

import rclpy
from object_detection.yolo_detector_base import YOLODetectorBase


class MalletDetectorNode(YOLODetectorBase):
    """Mallet-specific detector node."""
    
    def __init__(self):
        super().__init__('mallet_detector_node', 'mallet')


def main(args=None):
    rclpy.init(args=args)
    node = MalletDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
