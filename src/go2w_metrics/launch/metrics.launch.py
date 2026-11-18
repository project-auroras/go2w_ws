"""Hardware metrics of this machine, to record in the test bags.

ros2_vitals collector -> /vitals/status (CPU, RAM, temperatures, network, disk, NVIDIA GPU
with NVML, CPU/RAM of every ROS process; hostname tells which machine), and on the Jetson
also tegrastats -> /jetson/stats (GPU, EMC, thermal zones, power rails).

  NX:     ros2 launch go2w_metrics metrics.launch.py jetson:=true
  laptop: ros2 launch go2w_metrics metrics.launch.py      (scripts/record_tests.sh does it)
"""

import re
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    rate = ParameterValue(LaunchConfiguration('rate'), value_type=float)
    return LaunchDescription([
        DeclareLaunchArgument('rate', default_value='2.0', description='Hz of /vitals/status'),
        DeclareLaunchArgument('jetson', default_value='false',
                              description='true on the NX: also tegrastats -> /jetson/stats'),
        DeclareLaunchArgument('jetson_interval_ms', default_value='500'),

        # standalone collector: no root daemon, network per process only TCP (ss); the
        # node name includes the host so the two machines do not clash
        Node(package='ros2_vitals', executable='collector', output='screen',
             name='vitals_' + re.sub(r'\W', '_', socket.gethostname()),
             parameters=[{'publish_rate': rate, 'topic': '/vitals/status',
                          'enable_kill_service': False}]),

        Node(package='go2w_metrics', executable='jetson_stats', output='screen',
             parameters=[{'interval_ms': ParameterValue(
                 LaunchConfiguration('jetson_interval_ms'), value_type=int)}],
             condition=IfCondition(LaunchConfiguration('jetson'))),
    ])
