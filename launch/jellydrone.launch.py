from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joystick driver node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 20.0
            }],
            output='screen'
        ),

        # Joystick to servo command converter (publishes std_msgs/String)
        Node(
            package='jellydrone',
            executable='joystick_servo_control',
            name='joystick_servo_control',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 115200
                }],
            output='screen'
        ),

        # Serial bridge: sends commands & publishes IMU
        Node(
            package='jellydrone',
            executable='serial_bridge',
            name='serial_bridge',
            parameters=[{
                'port': '/dev/ttyACM0',     # ✅ Set this correctly!
                'baudrate': 115200
            }],
            output='screen'
        ),
    ])
