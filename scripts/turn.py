#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future

from geometry_msgs.msg import TwistStamped as Twist 
from nav_msgs.msg import Odometry 

from com_offer_holder_days_modules.tb3_tools import quaternion_to_euler
from math import degrees
import numpy as np

class Turn(Node):

    def __init__(self):
        super().__init__("turn_node")

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
            callback=self.exec_turn,
        )

        self.declare_parameter('angle', 45)
        self.yaw_ang_request = self.get_parameter('angle').get_parameter_value().integer_value
        self.turn_dir = np.sign(self.yaw_ang_request)
        self.yaw_ang_request = abs(self.yaw_ang_request)
        self.yaw_ang_request = min(self.yaw_ang_request, 360) # limit to 360 degrees
        self.do_not_move = True if self.yaw_ang_request == 0 else False

        self.theta_z = 0.0
        self.theta_zref = 0.0
        self.yaw = 0.0 # a variable to keep track of how far the robot has turned
             
        self.shutdown = False
        
        self.get_logger().info(
            f"Request to turn by {self.turn_dir * self.yaw_ang_request} degrees..."
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

        (_, _, yaw) = quaternion_to_euler(pose.orientation)

        self.theta_z = degrees(abs(yaw)) 

        if not self.first_message: 
            self.first_message = True
            self.theta_zref = self.theta_z

    def exec_turn(self):

        if self.hold:
            return
        
        # turn by X degrees...
        self.yaw = self.yaw + abs(self.theta_z - self.theta_zref)
        self.theta_zref = self.theta_z
        if self.yaw >= self.yaw_ang_request or self.do_not_move:
            # That's enough, stop turning!
            self.vel_msg = Twist()
            for i in range(5):
                self.vel_pub.publish(self.vel_msg)
            self.get_logger().info(
                f"Turning [{self.yaw:.0f}/{self.yaw_ang_request} degrees]."
            )
            self.yaw = 0.0
            self.done_future.set_result('done')
        else:
            # Not there yet, keep going:
            self.vel_msg.twist.angular.z = self.turn_dir * 0.3
            self.get_logger().info(
                f"Turning [{self.yaw:.0f}/{self.yaw_ang_request} degrees].",
                throttle_duration_sec=0.5,
            )    
            self.vel_pub.publish(self.vel_msg)

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    node = Turn()
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