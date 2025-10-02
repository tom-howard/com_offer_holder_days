#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future

from geometry_msgs.msg import TwistStamped as Twist
from nav_msgs.msg import Odometry 

from math import sqrt, pow

class MoveFwd(Node):

    def __init__(self):
        super().__init__("node")

        self.vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10,
        )

        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="odom",
            callback=self.odom_callback,
            qos_profile=10,
        )

        self.vel_msg = Twist()
        self.first_message = False 
        self.hold = True

        self.done_future = Future()
        
        ctrl_rate = 10 # hz
        self.timer = self.create_timer(
            timer_period_sec=1/ctrl_rate,
            callback=self.exec_fwd_motion,
        )

        self.declare_parameter('dist', 0.3)
        self.fwd_distance_request = self.get_parameter('dist').get_parameter_value().double_value

        self.x = 0.0; self.y = 0.0
        self.xref = 0.0; self.yref = 0.0
        self.distance = 0.0 # a variable to keep track of how far the robot has moved
             
        self.shutdown = False
        
        self.get_logger().info(
            f"Request to move forwards by {self.fwd_distance_request} meters..."
        )

        time.sleep(1.0)
        self.hold = False
        
    def on_shutdown(self):
        print("Stopping the robot...")
        for i in range(5):
            self.vel_pub.publish(Twist())
        self.shutdown = True

    def odom_callback(self, msg_data: Odometry):
        pose = msg_data.pose.pose 
        
        self.x = pose.position.x 
        self.y = pose.position.y

        if not self.first_message: 
            self.first_message = True
            self.xref = self.x
            self.yref = self.y

    def exec_fwd_motion(self):

        if self.hold:
            return
        
        self.distance = self.distance + sqrt(pow(self.x-self.xref, 2) + pow(self.y-self.yref, 2))
        self.xref = self.x
        self.yref = self.y
        if self.distance >= self.fwd_distance_request:
            # That's enough, stop moving!
            self.vel_msg = Twist()
            self.distance = 0.0
            self.get_logger().info(
                "Stopped."
            )
            self.done_future.set_result('done')
        else:
            # Not there yet, keep going:
            self.vel_msg.twist.linear.x = 0.1
            self.get_logger().info(
                f"Moving forwards [{self.distance:.2f}/{self.fwd_distance_request:.2f} m]."
            )

        # publish whatever velocity command has been set above:
        self.vel_pub.publish(self.vel_msg)

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = MoveFwd()
    try:
        rclpy.spin_until_future_complete(node, node.done_future)
    except KeyboardInterrupt:
        print(
            f"{node.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        node.on_shutdown()
        while not node.shutdown:
            continue
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
