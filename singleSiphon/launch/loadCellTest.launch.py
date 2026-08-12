# Minimal launch file for bringing up just the micro-ROS agent (so the
# load cell ESP32 can connect over WiFi and publish loadcell_data) plus an
# optional PlotJuggler and/or Foxglove bridge for looking at it live. No
# cameras, no actuator teleop, no rosbag — just enough to test the load
# cell board in isolation.
#
# Usage:
#   ros2 launch singleSiphon loadCellTest.launch.py
#   ros2 launch singleSiphon loadCellTest.launch.py plotjuggler:=true
#   ros2 launch singleSiphon loadCellTest.launch.py foxglove:=false

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    plotjuggler_arg = DeclareLaunchArgument(
        'plotjuggler', default_value='false',
        description='Launch PlotJuggler for topic visualization'
    )
    foxglove_arg = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Launch the Foxglove bridge for topic visualization'
    )

    return LaunchDescription([
        plotjuggler_arg,
        foxglove_arg,

        # micro-ROS agent — same one the load cell (and everything else)
        # connects to over WiFi.
        ExecuteProcess(
            cmd=[
                os.path.expanduser('~/uagent_ws/src/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent'),
                'udp4', '--port', '8888', '-v', '0'
            ],
            cwd=os.path.expanduser('~/uagent_ws/src/Micro-XRCE-DDS-Agent/build'),
            output='screen'
        ),

        # Optional PlotJuggler (pass plotjuggler:=true to enable). No
        # --layout here — the saved demo.xml layout is for the main robot's
        # actuator/camera topics, not loadcell_data.
        Node(
            package='plotjuggler',
            executable='plotjuggler',
            output='screen',
            additional_env={'QT_QPA_PLATFORM': 'xcb'},
            condition=IfCondition(LaunchConfiguration('plotjuggler')),
        ),

        # Foxglove bridge (pass foxglove:=false to disable). Default port
        # 8765 — connect Foxglove Studio's "Foxglove WebSocket" source to
        # ws://<this machine's IP>:8765.
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            condition=IfCondition(LaunchConfiguration('foxglove')),
        ),
    ])
