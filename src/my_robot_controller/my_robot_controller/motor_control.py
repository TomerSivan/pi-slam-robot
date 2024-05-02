#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorControlNode(Node):
    def __init__(self):
        super().__init__("motor_controller")
        self.get_logger().info("Initializing motor controller")

        # Define GPIO pins for motor control
        self.ENA_PIN1 = 27  # LEFT
        self.ENA_PIN2 = 25  # RIGHT
        self.IN1_PIN = 23   # LEFT
        self.IN2_PIN = 22   # LEFT
        self.IN3_PIN = 24   # RIGHT
        self.IN4_PIN = 17   # RIGHT

        # Setup GPIO pins
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.ENA_PIN1, GPIO.OUT)
        GPIO.setup(self.ENA_PIN2, GPIO.OUT)
        GPIO.setup(self.IN1_PIN, GPIO.OUT)
        GPIO.setup(self.IN2_PIN, GPIO.OUT)
        GPIO.setup(self.IN3_PIN, GPIO.OUT)
        GPIO.setup(self.IN4_PIN, GPIO.OUT)

        # Setup PWM for speed control
        self.pwm_left = GPIO.PWM(self.ENA_PIN1, 100)  # 100 Hz frequency
        self.pwm_right = GPIO.PWM(self.ENA_PIN2, 100)  # 100 Hz frequency
        self.pwm_left.start(0)  # Start with 0% duty cycle
        self.pwm_right.start(0)  # Start with 0% duty cycle

        # Create subscriber to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription

    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities from Twist message
        linear_vel_x = msg.linear.x * 100
        angular_vel_z = msg.angular.z * 100

        # Convert linear and angular velocities to motor speeds
        left_speed = linear_vel_x - angular_vel_z
        right_speed = linear_vel_x + angular_vel_z

        # Ensure motor speeds are within valid range (-100 to 100)
        left_speed = max(-100, min(100, left_speed))
        right_speed = max(-100, min(100, right_speed))

        # Set motor directions and speeds
        self.set_motor_speed(self.IN1_PIN, self.IN2_PIN, self.pwm_left, left_speed)
        self.set_motor_speed(self.IN3_PIN, self.IN4_PIN, self.pwm_right, right_speed)

    def set_motor_speed(self, pin1, pin2, pwm, speed):
        if speed > 0:
            GPIO.output(pin1, GPIO.HIGH)
            GPIO.output(pin2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(pin1, GPIO.LOW)
            GPIO.output(pin2, GPIO.HIGH)
        else:
            GPIO.output(pin1, GPIO.LOW)
            GPIO.output(pin2, GPIO.LOW)
        pwm.ChangeDutyCycle(abs(speed))

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    GPIO.cleanup()
    rclpy.shutdown()

if __name__ == '__main__':
    main()