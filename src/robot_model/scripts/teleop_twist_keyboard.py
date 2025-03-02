#!/usr/bin/env python3

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String, Float64MultiArray
from geometry_msgs.msg import Twist
from rclpy import qos

class keyboard_control(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        self.keyboard_pub = self.create_publisher(
            String, 'keyboard_input', qos_profile=qos.qos_profile_system_default
        )
        
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', qos_profile=qos.qos_profile_system_default
        )
        
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, 'gripper_controller/commands', qos_profile=qos.qos_profile_system_default
        )

        
        self.moveSpeed: float = 0.0
        self.slideSpeed: float = 0.0
        self.turnSpeed: float = 0.0
        self.plusMoveSpeed: float = 0.25
        self.plusSlideSpeed: float = 0.25
        self.plusturnSpeed: float = 0.5
        self.plusSpeedSize: float = 0.01
        self.maxSpeed : float = 3.0 # m/s
        
        self.gripperSlideVel: float = 0.0
        self.gripperUpDownVel: float = 0.0
        self.maxGripperSlideVel: float = 0.5
        self.maxGripperUpDownVel: float = 0.5
        self.gripperIncrement: float = 0.1
        
        self.show_log()
        self.get_logger().info("Keyboard publisher started. Press keys to send messages. Press 'p' to quit.")


    def clip(self, value : float, min_val : float, max_val : float) -> float:
        return max(min(value, max_val), min_val)

    def show_log(self):
        log_message = (
            "Use the following keys to control the robot: \n"
            "'W' : Increase move speed, \n"
            "'S' : Decrease move speed, \n"
            "'A' : Slide left, \n"
            "'D' : Slide right, \n"
            "'Q' : Turn left, \n"
            "'E' : Turn right, \n"
            "'y' : Increase move speed increment. \n"
            "'h' : Decrease move speed increment. \n"
            "'u' : Increase slide speed increment. \n"
            "'j' : Decrease slide speed increment. \n"
            "'i' : Increase turn speed increment. \n"
            "'k' : Decrease turn speed increment. \n"
            "' ' : Brake. \n"
            "'8' : Slide Gripper Up. \n"
            "'2' : Slide Gripper Down. \n"
            "'4' : Close Gripper. \n"
            "'6' : Open Gripper Up. \n"
            "'5' : Set to defalt position. \n"
            "Keyboard publisher started. Press keys to send messages. Press 'p' to quit.\n"
            f"Current Speeds: Move={self.moveSpeed:.2f}, Slide={self.slideSpeed:.2f}, Turn={self.turnSpeed:.2f}\n"
            f"Current Speed increment: Move={self.plusMoveSpeed:.2f}, Slide={self.plusSlideSpeed:.2f}, Turn={self.plusturnSpeed:.2f}"
            f"\nCurrent Gripper Velocity: Slide={self.gripperSlideVel:.2f}, Up/Down={self.gripperUpDownVel:.2f}"
        )

        # Log the combined message
        self.get_logger().info(log_message)
        
    def update_speeds(self, key):
        # Adjust movement, sliding, and turning speeds
        self.moveSpeed = self.moveSpeed + self.plusMoveSpeed if key == 'w' else self.moveSpeed - self.plusMoveSpeed if key == 's' else self.moveSpeed
        self.slideSpeed = self.slideSpeed + self.plusSlideSpeed if key == 'd' else self.slideSpeed - self.plusSlideSpeed if key == 'a' else self.slideSpeed
        self.turnSpeed = self.turnSpeed + self.plusturnSpeed if key == 'q' else self.turnSpeed - self.plusturnSpeed if key == 'e' else self.turnSpeed

        self.plusMoveSpeed = self.plusMoveSpeed + self.plusSpeedSize if key == 'W' else self.plusMoveSpeed - self.plusSpeedSize if key == 'S' else self.plusMoveSpeed
        self.plusSlideSpeed = self.plusSlideSpeed + self.plusSpeedSize if key == 'A' else self.plusSlideSpeed - self.plusSpeedSize if key == 'D' else self.plusSlideSpeed
        self.plusturnSpeed = self.plusturnSpeed + self.plusSpeedSize if key == 'Q' else self.plusturnSpeed - self.plusSpeedSize if key == 'E' else self.plusturnSpeed

        if key == ' ':
            self.moveSpeed = 0.0
            self.slideSpeed = 0.0
            self.turnSpeed = 0.0

        if key == '4' or key == '6' or key == '8' or key == '2' or key == '5':
            self.gripperSlideVel = self.gripperSlideVel + self.gripperIncrement if key == '4' else self.gripperSlideVel - self.gripperIncrement if key == '6' else self.gripperSlideVel
            self.gripperUpDownVel = self.gripperUpDownVel + self.gripperIncrement if key == '8' else self.gripperUpDownVel - self.gripperIncrement if key == '2' else self.gripperUpDownVel

            self.gripperSlideVel = self.clip(self.gripperSlideVel, -self.maxGripperSlideVel, self.maxGripperSlideVel)
            self.gripperUpDownVel = self.clip(self.gripperUpDownVel, -self.maxGripperUpDownVel, self.maxGripperUpDownVel)
            
            if key == '5':  # Reset Gripper to default position
                self.gripperSlideVel = 0.0
                self.gripperUpDownVel = 0.0
                
            gripper_vel_msg = Float64MultiArray()
            gripper_vel_msg.data = [float(self.gripperUpDownVel), float(self.gripperSlideVel), float(-self.gripperSlideVel)]
            self.gripper_pub.publish(gripper_vel_msg)
                

        # Clip values
        self.moveSpeed = self.clip(self.moveSpeed, -self.maxSpeed, self.maxSpeed)
        self.slideSpeed = self.clip(self.slideSpeed, -self.maxSpeed, self.maxSpeed)
        self.turnSpeed = self.clip(self.turnSpeed, -self.maxSpeed, self.maxSpeed)
        self.plusMoveSpeed = self.clip(self.plusMoveSpeed, 0, self.maxSpeed)
        self.plusSlideSpeed = self.clip(self.plusSlideSpeed, 0, self.maxSpeed)
        self.plusturnSpeed = self.clip(self.plusturnSpeed, 0, self.maxSpeed)
        
    def get_key(self):
        """Reads a single key without waiting for Enter."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            self.show_log()
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def run(self):
        """Continuously listens for keypresses and publishes them."""
        while rclpy.ok():
            key = self.get_key()
            if key == 'p':  # Exit condition
                break
            self.update_speeds(key)
            self.publish_message(key)

    def publish_message(self, key):
        keyboard_msg = String()
        keyboard_msg.data = key
        
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x    = float(self.moveSpeed)
        cmd_vel_msg.linear.y    = float(self.slideSpeed)
        cmd_vel_msg.angular.z   = float(self.turnSpeed)
        
        
        self.keyboard_pub.publish(keyboard_msg)
        self.cmd_vel_pub.publish(cmd_vel_msg)

def main():
    rclpy.init()
    node = keyboard_control()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()