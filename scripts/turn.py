#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future

from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry 

from com_offer_holder_days_modules.tb3_tools import quaternion_to_euler
from math import degrees

class Square(Node):

    def __init__(self):
        super().__init__("move_square")

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
        self.turn = False 
        
        self.done_future = Future()

        ctrl_rate = 10 # hz
        self.timer = self.create_timer(
            timer_period_sec=1/ctrl_rate,
            callback=self.timer_callback,
        )

        self.declare_parameter('yaw_ang', 45)
        self.yaw_ang_request = self.get_parameter('yaw_ang').get_parameter_value().integer_value

        self.x = 0.0; self.y = 0.0; self.theta_z = 0.0
        self.xref = 0.0; self.yref = 0.0; self.theta_zref = 0.0
        self.yaw = 0.0 # a variable to keep track of how far the robot has turned
        self.displacement = 0.0 # a variable to keep track of how far the robot has moved
             
        self.shutdown = False
        
        self.get_logger().info(
            f"Request to turn by {self.yaw_ang_request} degrees..."
        )

    def on_shutdown(self):
        print("Stopping the robot...")
        self.vel_pub.publish(Twist())
        self.shutdown = True

    def odom_callback(self, msg_data: Odometry):
        pose = msg_data.pose.pose 
        
        self.x = pose.position.x 
        self.y = pose.position.y

        (roll, pitch, yaw) = quaternion_to_euler(pose.orientation)

        self.theta_z = degrees(abs(yaw)) # abs(yaw) makes life much easier!!

        if not self.first_message: 
            self.first_message = True
            self.xref = self.x
            self.yref = self.y
            self.theta_zref = self.theta_z

    def timer_callback(self):
        # turn by X degrees...
        # keep track of how much yaw has been accrued during the current turn
        self.yaw = self.yaw + abs(self.theta_z - self.theta_zref)
        self.theta_zref = self.theta_z
        if self.yaw >= self.yaw_ang_request:
            # That's enough, stop turning!
            self.vel_msg = Twist()
            self.turn = False
            self.yaw = 0.0
            self.xref = self.x
            self.yref = self.y
            self.get_logger().info(
                "Stopped."
            )
            self.done_future.set_result('done')
        else:
            # Not there yet, keep going:
            self.vel_msg.angular.z = 0.3
            self.get_logger().info(
                "Turning.."
            )
    

        # publish whatever velocity command has been set above:
        self.vel_pub.publish(self.vel_msg)

def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    move_square = Square()
    try:
        rclpy.spin_until_future_complete(move_square, move_square.done_future)
    except KeyboardInterrupt:
        print(
            f"{move_square.get_name()} received a shutdown request (Ctrl+C)."
        )
    finally:
        move_square.on_shutdown()
        while not move_square.shutdown:
            continue
        move_square.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()