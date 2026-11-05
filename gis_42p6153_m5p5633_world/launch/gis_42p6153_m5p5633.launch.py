# Copyright 2026 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch gis_42p6153_m5p5633_world standalone in Gazebo (gz sim -r -v4), no ROS robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world_file_name = 'gis_42p6153_m5p5633.world'
    package_dir = get_package_share_directory('gis_42p6153_m5p5633_world')
    world = LaunchConfiguration('world')

    model_path = os.path.join(package_dir, 'models')

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v4', world], output='screen')

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path),
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(package_dir, 'worlds', world_file_name), ''],
            description='SDF world file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='true'),
        gazebo,
    ])


if __name__ == '__main__':
    generate_launch_description()
