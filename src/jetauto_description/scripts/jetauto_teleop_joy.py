#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition, BuzzerState, MotorsState
from controller import mecanum

# Index map based on documentation
class GamepadMap:
    LX = 0
    LY = 1
    RX = 2
    RY = 3
    R2 = 4
    L2 = 5
    DX = 6
    DY = 7

    A = 0
    B = 1
    X = 3
    Y = 4
    L1 = 6
    R1 = 7
    SELECT = 10
    START = 11
    L3 = 13
    R3 = 14

class JetAutoTeleopJoy(Node):
    def __init__(self):
        super().__init__('jetauto_teleop_joy')

        # Parameters
        self.declare_parameter('max_linear_vel', 0.5)  # m/s
        self.declare_parameter('max_angular_vel', 1.0) # rad/s
        self.declare_parameter('servo_speed_step', 15)

        self.max_lin = self.get_parameter('max_linear_vel').value
        self.max_ang = self.get_parameter('max_angular_vel').value
        self.servo_step = self.get_parameter('servo_speed_step').value

        self.mecanum = mecanum.MecanumChassis()

        # Internal State
        self.cam_pan_pos = 500
        self.last_msg_had_movement = False
        self.last_buzzer_state = 0

        # Publishers
        self.pub_motor = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 10)
        self.pub_servo = self.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10)
        self.pub_buzzer = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 10)

        # Subscriber
        self.sub_joy = self.create_subscription(Joy, '/ros_robot_controller/joy', self.joy_callback, 10)
        
        print('\n\033[92m[ OK ] - JetAuto Teleop Node Started Successfully.\033[0m\n')

    def joy_callback(self, msg):
        self.handle_base_movement(msg)
        self.handle_camera_pan(msg)
        self.handle_buzzer(msg)

    def handle_base_movement(self, msg):
        # E-STOP
        if msg.buttons[GamepadMap.START] == 1 and msg.buttons[GamepadMap.SELECT] == 1:
            self.stop_motors()
            return

        # R2 turbo, L2 precision
        speed_multiplier = 1.0
        if msg.axes[GamepadMap.R2] > 0.1:
            speed_multiplier = 2.0  # Turbo
        elif msg.axes[GamepadMap.L2] > 0.1:
            speed_multiplier = 0.25 # Turtle mode
            
        # Left Stick = Translation (X forward/back, Y left/right mecanum strafe)
        # Note: In ROS joy, up is +1.0, down is -1.0. Right is -1.0, left is +1.0
        lx = msg.axes[GamepadMap.LY] * self.max_lin * speed_multiplier
        ly = msg.axes[GamepadMap.LX] * self.max_lin * speed_multiplier
        
        # Right Stick = Rotation (Z yaw)
        az = msg.axes[GamepadMap.RX] * self.max_ang * speed_multiplier

        # Deadzone logic to prevent micro-drifting
        if abs(lx) < 0.05: lx = 0.0
        if abs(ly) < 0.05: ly = 0.0
        if abs(az) < 0.05: az = 0.0

        moving = (lx != 0.0 or ly != 0.0 or az != 0.0)

        if moving or self.last_msg_had_movement:
            motor_cmd = self.mecanum.set_velocity(lx, ly, az)
            self.pub_motor.publish(motor_cmd)
            self.last_msg_had_movement = moving

    def stop_motors(self):
        self.pub_motor.publish(self.mecanum.set_velocity(0.0, 0.0, 0.0))
        self.last_msg_had_movement = False

    def handle_camera_pan(self, msg):
        position_changed = False
        
        # D-Pad for slow panning
        if msg.axes[GamepadMap.DX] > 0.5: # Left
            self.cam_pan_pos += self.servo_step
            position_changed = True
        elif msg.axes[GamepadMap.DX] < -0.5: # Right
            self.cam_pan_pos -= self.servo_step
            position_changed = True

        # L3 for instant center
        if msg.buttons[GamepadMap.L3] == 1:
            self.cam_pan_pos = 500
            position_changed = True

        # Constrain limits 200 to 800
        if self.cam_pan_pos > 800: self.cam_pan_pos = 800
        if self.cam_pan_pos < 200: self.cam_pan_pos = 200

        if position_changed:
            servo_cmd = ServosPosition()
            servo_cmd.duration = 0.05 # Fast react
            sv = ServoPosition()
            sv.id = 1
            sv.position = self.cam_pan_pos
            servo_cmd.position = [sv]
            self.pub_servo.publish(servo_cmd)

    def handle_buzzer(self, msg):
        current_l1 = msg.buttons[GamepadMap.L1]
        
        # Trigger on rising edge (just pressed)
        if current_l1 == 1 and self.last_buzzer_state == 0:
            buzzer_msg = BuzzerState()
            buzzer_msg.freq = 2500
            buzzer_msg.on_time = 0.1
            buzzer_msg.off_time = 0.1
            buzzer_msg.repeat = 1
            self.pub_buzzer.publish(buzzer_msg)
            
        self.last_buzzer_state = current_l1


def main(args=None):
    print("Starting Main...")
    try:
        rclpy.init(args=args)
        print("RCLPY Initiated")
        node = JetAutoTeleopJoy()
        print("Node Object Created")
        rclpy.spin(node)
    except Exception as e:
        print(f"CRITICAL ERROR: {e}")
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()
