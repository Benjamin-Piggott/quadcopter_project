#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from quadcopter_interfaces.srv import FindPath
from std_msgs.msg import Float32MultiArray

class MotorControlClient(Node):
    def __init__(self):
        super().__init__('motor_control_client_node')
        
        # Subscriber for path
        self.create_subscription(
            Float32MultiArray,
            '/quadcopter/optimal_path',
            self.path_callback,
            10
        )
        
        # Motor control publishers
        self.motor_pubs = {
            'front_left': self.create_publisher(Float32MultiArray, '/quadcopter/motor/front_left', 10),
            'front_right': self.create_publisher(Float32MultiArray, '/quadcopter/motor/front_right', 10),
            'rear_left': self.create_publisher(Float32MultiArray, '/quadcopter/motor/rear_left', 10),
            'rear_right': self.create_publisher(Float32MultiArray, '/quadcopter/motor/rear_right', 10)
        }
        
        self.get_logger().info("Motor Control Client Node Initialized")
    
    def path_callback(self, path_msg):
        """
        Callback to process received path and generate motor commands
        """
        # Convert path to 2D coordinates
        path = [(path_msg.data[i], path_msg.data[i+1]) for i in range(0, len(path_msg.data), 2)]
        
        # Generate motor commands based on path
        motor_commands = self.generate_motor_commands(path)
        
        # Publish motor commands
        for motor, cmd in motor_commands.items():
            motor_msg = Float32MultiArray()
            motor_msg.data = cmd
            self.motor_pubs[motor].publish(motor_msg)
    
    def generate_motor_commands(self, path):
        """
        Generate motor commands based on path
        Simple implementation - adjust based on actual quadcopter dynamics
        """
        return {
            'front_left': [0.5, 0.0],
            'front_right': [0.5, 0.0],
            'rear_left': [0.5, 0.0],
            'rear_right': [0.5, 0.0]
        }

def main(args=None):
    rclpy.init(args=args)
    motor_control_client = MotorControlClient()
    try:
        rclpy.spin(motor_control_client)
    except KeyboardInterrupt:
        pass
    finally:
        motor_control_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()