from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from simulation.movement_subscriber import MovementSubscriber
from rclpy.executors import MultiThreadedExecutor

class StatePublisher(Node):

    def __init__(self):
        rclpy.init()
        super().__init__('state_publisher')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        self.movement_subscriber = MovementSubscriber()
        degree = pi / 180.0
        # Define 4 continuous wheels

        self.joint_state = JointState()
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'base_link'

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0
        self.pitch = 0
        self.roll = 0
        
        self.loop_rate = self.create_rate(10)  # Lower the rate (e.g., 10 Hz or even 5 Hz)
   
    def start_simulation_rotation(self):
        executor = MultiThreadedExecutor()
        executor.add_node(self.movement_subscriber)

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                executor.spin_once(timeout_sec=0.1)

                now = self.get_clock().now()
                self.joint_state.header.stamp = now.to_msg()
                
                # Fetch movement data as a single dictionary
                data = self.movement_subscriber.get_movement_data()  # Returns {'left_speed': 1, 'right_speed': 1, 'angle': 90}
                wheel_angle = None
                if data == None:
                    continue
                left_speed = data["left_speed"]
                right_speed = data["right_speed"]
                if data["angle"] != None:
                    wheel_angle = data["angle"] * (pi / 180)  # Convert angle from degrees to radians
                else:
                    wheel_angle = None

                if wheel_angle == None:
                    wheel_angle = 0.0
                # Define joint names for the wheels
                self.joint_state.name = [
                    'base_left_front_wheel_joint',
                    'base_right_front_joint',
                    'base_left_back_joint',
                    'base_right_back_joint'
                ]

                WHEEL_RADIUS = 0.0325  
                WHEELBASE = 0.12  

                linear_velocity = (left_speed + right_speed) 
                angular_velocity = (right_speed - left_speed) / WHEELBASE

                # Calculate position change
                delta_x = cos(self.yaw) * linear_velocity * 0.05  
                delta_y = sin(self.yaw) * linear_velocity * 0.05
                delta_yaw = angular_velocity * 0.01

                # Update pose
                self.x += delta_x
                self.y += delta_y
                self.yaw += delta_yaw
                self.yaw %= (2 * pi) 

                wheel_angular_velocity_left = left_speed / WHEEL_RADIUS  
                wheel_angular_velocity_right = right_speed / WHEEL_RADIUS  

                self.joint_state.position = [self.yaw, self.yaw, self.yaw, self.yaw]
                self.joint_state.velocity = [
                    wheel_angular_velocity_left,
                    wheel_angular_velocity_right,
                    wheel_angular_velocity_left,
                    wheel_angular_velocity_right,
                ]
                self.joint_state.effort = [0.0, 0.0, 0.0, 0.0]

                # Update transform to simulate movement
                self.odom_trans.header.stamp = now.to_msg()
                self.odom_trans.transform.translation.x += delta_x
                self.odom_trans.transform.translation.y += delta_y
                self.odom_trans.transform.translation.z = 0.0 

                # Update the rotation quaternion based on yaw
                self.odom_trans.transform.rotation = self.euler_to_quaternion()

                # Publish joint states and transform
                self.joint_pub.publish(self.joint_state)
                self.broadcaster.sendTransform(self.odom_trans)


                self.loop_rate.sleep()

        except KeyboardInterrupt:
            pass

    def euler_to_quaternion(self):
        # Convert Euler angles to quaternion for the transform
        q = Quaternion()
        q.x = sin(self.roll / 2) * cos(self.pitch / 2) * cos(self.yaw / 2) - cos(self.roll / 2) * sin(self.pitch / 2) * sin(self.yaw / 2)
        q.y = cos(self.roll / 2) * sin(self.pitch / 2) * cos(self.yaw / 2) + sin(self.roll / 2) * cos(self.pitch / 2) * sin(self.yaw / 2)
        q.z = cos(self.roll / 2) * cos(self.pitch / 2) * sin(self.yaw / 2) - sin(self.roll / 2) * sin(self.pitch / 2) * cos(self.yaw / 2)
        q.w = cos(self.roll / 2) * cos(self.pitch / 2) * cos(self.yaw / 2) + sin(self.roll / 2) * sin(self.pitch / 2) * sin(self.yaw / 2)
        return q


def main():
    state_publisher = StatePublisher()
    state_publisher.start_simulation_rotation()
if __name__ == '__main__':
    main()
