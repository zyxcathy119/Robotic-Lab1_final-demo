#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TimeBasedController(Node):
    def __init__(self):
        super().__init__('time_based_controller')
        self.debug = True

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.motion_move = Twist()
        self.motion_turn_right = Twist()
        self.motion_turn_left = Twist()
        self.motion_stop = Twist()

        self.motion_move.linear.x = 0.12
        self.motion_stop.linear.x = 0.0
        self.motion_turn_left.angular.z = 0.15
        self.motion_turn_right.angular.z = -0.15

        self.state = 'turn1'
        self.go_duration1 = 9.5      
        self.go_duration2 = 14.7    
        self.go_duration3 = 12.0  
        self.turn_duration1 = 5.0 
        self.turn_duration2 = 10.4
        self.turn_duration3 = 10.5

        self.state_start_time = self.get_clock().now().nanoseconds / 1e9

        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self.state_start_time
        print(self.state)

        actions = {
            'turn1': {
                'duration': self.turn_duration1,
                'continue_cmd': self.motion_turn_right,
                'transition_cmd': self.motion_move,
                'next_state': 'go1',
                'transition_log': f"Turn complete. Stopping and waiting for {self.go_duration1}s...",
                'debug_prefix': "Turning... Elapsed: {elapsed:.2f}s"
            },
            'go1': {
                'duration': self.go_duration1,
                'continue_cmd': self.motion_move,
                'transition_cmd': self.motion_turn_left,
                'next_state': 'turn2',
                'transition_log': f"Forward motion complete. Stopping and waiting for {self.turn_duration1}s...",
                'debug_prefix': "Moving forward... Elapsed: {elapsed:.2f}s"
            },
            'turn2': {
                'duration': self.turn_duration2,
                'continue_cmd': self.motion_turn_left,
                'transition_cmd': self.motion_move,
                'next_state': 'go2',
                'transition_log': f"Turn complete. Stopping and waiting for {self.go_duration2}s...",
                'debug_prefix': "Turning... Elapsed: {elapsed:.2f}s"
            },
            'go2': {
                'duration': self.go_duration2,
                'continue_cmd': self.motion_move,
                'transition_cmd': self.motion_turn_right,
                'next_state': 'turn3',
                'transition_log': f"Forward motion complete. Stopping and waiting for {self.turn_duration2}s...",
                'debug_prefix': "Moving forward... Elapsed: {elapsed:.2f}s"
            },
            'turn3': {
                'duration': self.turn_duration3,
                'continue_cmd': self.motion_turn_right,
                'transition_cmd': self.motion_move,
                'next_state': 'go3',
                'transition_log': f"Turn complete. Stopping and waiting for {self.go_duration3}s...",
                'debug_prefix': "Turning... Elapsed: {elapsed:.2f}s"
            },
            'go3': {
                'duration': self.go_duration3,
                'continue_cmd': self.motion_move,
                'transition_cmd': self.motion_stop,
                'next_state': None,
                'transition_log': "Forward motion complete. Stopping...",
                'debug_prefix': "Moving forward... Elapsed: {elapsed:.2f}s"
            }
        }

        if self.state not in actions:
            return

        action = actions[self.state]
        if elapsed < action['duration']:
            self.pub.publish(action['continue_cmd'])
            if self.debug:
                self.get_logger().info(action['debug_prefix'].format(elapsed=elapsed))
        else:
            self.pub.publish(action['transition_cmd'])
            self.get_logger().info(action['transition_log'])
            if action['next_state'] is None:
                rclpy.shutdown()
            else:
                self.state = action['next_state']
                self.state_start_time = now

def main(args=None):
    time.sleep(130)
    rclpy.init(args=args)
    node = TimeBasedController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
