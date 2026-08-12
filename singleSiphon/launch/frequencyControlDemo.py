# Frequency Control Demo Launch File
#
# Basic:
#   ros2 launch singleSiphon frequencyControlDemo.py
#
# With serial debug monitor (relay ESP32 on /dev/ttyACM1 at 921600):
#   ros2 launch singleSiphon frequencyControlDemo.py debug:=true
#
# With a different debug serial port or baud rate:
#   ros2 launch singleSiphon frequencyControlDemo.py debug:=true serial_port:=/dev/ttyACM0 baud_rate:=115200
#
# With rosbag recording (saves to ~/SIPHION_Master_Folder/esp32_bags/run_<timestamp>/):
#   ros2 launch singleSiphon frequencyControlDemo.py rosbag:=true
#
# With PlotJuggler:
#   ros2 launch singleSiphon frequencyControlDemo.py plotjuggler:=true
#
# Everything at once:
#   ros2 launch singleSiphon frequencyControlDemo.py debug:=true rosbag:=true plotjuggler:=true

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
import os
from datetime import datetime

def generate_launch_description():
    debug_arg = DeclareLaunchArgument(
        'debug', default_value='false',
        description='Open a serial debug monitor in xterm'
    )
    serial_port_arg = DeclareLaunchArgument(
        'serial_port', default_value='/dev/ttyACM1',
        description='Serial port for the debug monitor (relay ESP32 USB CDC)'
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate', default_value='921600',
        description='Baud rate for the debug serial port — must match SerialDebug UART rate on the relay ESP32'
    )
    plotjuggler_arg = DeclareLaunchArgument(
        'plotjuggler', default_value='false',
        description='Launch PlotJuggler for topic visualization'
    )
    rosbag_arg = DeclareLaunchArgument(
        'rosbag', default_value='false',
        description='Record a rosbag to ~/SIPHION_Master_Folder/esp32_bags/run_<timestamp>/'
    )

    bag_path = os.path.expanduser(
        f'~/SIPHION_Master_Folder/esp32_bags/run_{datetime.now().strftime("%Y%m%d_%H%M%S")}'
    )



    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge'
    )

    return LaunchDescription([
        debug_arg,
        serial_port_arg,
        baud_rate_arg,
        plotjuggler_arg,
        rosbag_arg,
        foxglove,

        # micro-ROS agent
        ExecuteProcess(
            cmd=[
                os.path.expanduser('~/uagent_ws/src/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent'),
                'udp4', '--port', '8888', '-v', '0'
            ],
            cwd=os.path.expanduser('~/uagent_ws/src/Micro-XRCE-DDS-Agent/build'),
            output='screen'
        ),

        # Teleop node
        Node(
            package='singleSiphon',
            executable='actuator_teleop',
            output='screen',
            prefix='xterm -geometry +1058+1295 -e',
        ),

        # Optional PlotJuggler (pass plotjuggler:=true to enable)
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            output='screen',
            additional_env={'QT_QPA_PLATFORM': 'xcb'},
            arguments=(
                ['--layout', os.path.expanduser('~/singleSiphon/config/demo.xml')]
                if os.path.exists(os.path.expanduser('~/singleSiphon/config/demo.xml'))
                else []
            ),
            condition=IfCondition(LaunchConfiguration('plotjuggler')),
        ),



        # Optional rosbag recording (pass rosbag:=true to enable).
        # Saves to ~/SIPHION_Master_Folder/esp32_bags/run_<timestamp>/
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', bag_path,
                'micro_ros/current_sensor',
                'micro_ros/encoder_position',
                'micro_ros/desired_position',
                'actuator_1/freq',
                'actuator_1/mode',
                'actuator_1/setpoint',
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('rosbag')),
        ),

        # Optional serial debug monitor (pass debug:=true to enable).
        # Reads from the relay ESP32 (wired to SerialDebug UART on D8/D9) and logs to
        # ~/SIPHION_Master_Folder/esp32_logs/esp32_debug_<timestamp>.log while showing live output in xterm.
        ExecuteProcess(
            cmd=[
                'xterm', '-title', 'Serial Debug', '-geometry', '+1044+881', '-e',
                'bash', '-c',
                [
                    'mkdir -p $HOME/SIPHION_Master_Folder/esp32_logs && '
                    'LOG=$HOME/SIPHION_Master_Folder/esp32_logs/esp32_debug_$(date +%Y%m%d_%H%M%S).log && '
                    'echo "Logging to: $LOG" && '
                    'fuser -k ', LaunchConfiguration('serial_port'), ' 2>/dev/null; '
                    'sleep 0.3; '
                    'screen -L -Logfile $LOG ', LaunchConfiguration('serial_port'), ' ', LaunchConfiguration('baud_rate'), '; '
                    'echo "Log saved to: $LOG  -- press Enter to close"; read'
                ]
            ],
            output='screen',
            condition=IfCondition(LaunchConfiguration('debug')),
        ),

    ])
