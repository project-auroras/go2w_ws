from glob import glob

from setuptools import find_packages, setup

package_name = 'go2w_metrics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@todo.todo',
    description='Hardware and ROS metrics of the test recordings, bags to CSV',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'jetson_stats = go2w_metrics.jetson_stats:main',
            'bag_to_csv = go2w_metrics.bag_to_csv:main',
        ],
    },
)
