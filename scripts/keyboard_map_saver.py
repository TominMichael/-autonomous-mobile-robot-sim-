#!/usr/bin/env python3

import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String

class KeyboardMapSaver(Node):
    def __init__(self):
        super().__init__('keyboard_map_saver')
        self.cli = self.create_client(SaveMap, '/slam_toolbox/save_map')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /slam_toolbox/save_map not available, waiting...')
            
        self.get_logger().info('Map Saver Ready!')
        self.get_logger().info('Press "s" to save the map ("my_map").')
        self.get_logger().info('Press "q" or "Ctrl+C" to quit.')

    def save_map(self, map_name="my_map"):
        req = SaveMap.Request()
        req.name = String()
        req.name.data = map_name
        
        self.get_logger().info(f'Saving map as {map_name}...')
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        try:
            response = future.result()
            if response.result == 0: # RESULT_SUCCESS
                 self.get_logger().info('Map saved successfully!')
            else:
                 self.get_logger().error(f'Failed to save map. Result code: {response.result}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {r}')

def get_key():
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    saver = KeyboardMapSaver()

    try:
        while rclpy.ok():
            key = get_key()
            if key == 's':
                saver.save_map("my_map")
            elif key == 'q' or key == '\x03': # q or ctrl-c
                break
            
            # Spin briefly to keep ROS communications alive
            rclpy.spin_once(saver, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        saver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
