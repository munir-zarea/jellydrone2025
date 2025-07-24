from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1) Joystick driver
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

        # 2) Joystick-to-servo control
        Node(
            package='jellydrone',
            executable='joystick_servo_control',
            name='joystick_servo_control',
            output='screen'
        ),

        # 3) Serial bridge to Arduino
        Node(
            package='jellydrone',
            executable='serial_bridge',
            name='serial_bridge',
            parameters=[{
                'port': '/dev/ttyACM0',
                'baudrate': 115200
            }],
            output='screen'
        ),
        # 4) Camera feed publisher
        Node(
            package='jellydrone',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'camera_index': 4  # U20CAM-720P is at /dev/video4
            }],
            output='screen'
        ),


    ])
