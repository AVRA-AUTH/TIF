#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class GameManager(Node):
    def __init__(self):
        super().__init__('game_manager')
        self.get_logger().info('Game Manager Node initialized.')
        # TODO: Manage game states (Setup, Run, Reset)
        # TODO: Track score or time

def main(args=None):
    rclpy.init(args=args)
    node = GameManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
