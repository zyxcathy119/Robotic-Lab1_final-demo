#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TimeBasedController(Node):
    def __init__(self):
        super().__init__('time_based_controller')
        # Debug flag to enable debug logging
        self.debug = True

        # Publisher to send velocity commands on the /cmd_vel topic
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Define Twist messages for moving forward, stopping, and moving backward
        self.motion_move = Twist()
        self.motion_stop = Twist()
        self.motion_return = Twist()

        # Set forward speed (0.12 m/s)
        self.motion_move.linear.x = 0.12
        # Set stop speed (0.0 m/s)
        self.motion_stop.linear.x = 0.0
        # Set reverse speed (-0.12 m/s)
        self.motion_return.linear.x = -0.12

        # Define state machine states: "go", "wait", "return"
        self.state = 'go'

        # Define durations in seconds for each phase:
        self.go_duration = 28.0      # Move forward for 28 seconds
        self.wait_duration = 100.0    # Wait for 100 seconds
        self.return_duration = 27.0  # Move backward for 27 seconds

        # Record the start time of the current state
        self.state_start_time = self.get_clock().now().nanoseconds / 1e9

        # Create a timer to periodically execute the control loop
        self.timer = self.create_timer(0.1, self.control_loop)

    def control_loop(self):
        """
        Control loop based on time.
        """
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.state_start_time

        if self.state == 'go':
            # While elapsed time is less than go_duration, keep moving forward.
            if elapsed_time < self.go_duration:
                self.pub.publish(self.motion_move)
                if self.debug:
                    self.get_logger().info(f"Moving forward... Elapsed: {elapsed_time:.2f}s")
            else:
                # Transition to waiting state.
                self.pub.publish(self.motion_stop)
                self.get_logger().info(f"Forward motion complete. Stopping and waiting for {self.wait_duration}s...")
                self.state = 'wait'
                self.state_start_time = current_time

        elif self.state == 'wait':
            # While elapsed time is less than wait_duration, keep waiting.
            if elapsed_time < self.wait_duration:
                self.pub.publish(self.motion_stop)
                if self.debug:
                    self.get_logger().info(f"Waiting... Elapsed: {elapsed_time:.2f}s")
            else:
                # Transition to return state.
                self.pub.publish(self.motion_return)
                self.get_logger().info("Wait complete. Moving backward for 30s...")
                self.state = 'return'
                self.state_start_time = current_time

        elif self.state == 'return':
            # While elapsed time is less than return_duration, keep moving backward.
            if elapsed_time < self.return_duration:
                self.pub.publish(self.motion_return)
                if self.debug:
                    self.get_logger().info(f"Returning... Elapsed: {elapsed_time:.2f}s")
            else:
                # After return_duration, stop the robot and finish the mission.
                self.pub.publish(self.motion_stop)
                self.get_logger().info("Return motion complete. Mission finished.")
                rclpy.shutdown()

def main(args=None):
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
