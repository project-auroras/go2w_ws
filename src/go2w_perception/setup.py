from glob import glob

from setuptools import find_packages, setup

package_name = 'go2w_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='unitree@todo.todo',
    description='Detección de personas con YOLO26 sobre la RealSense del Go2-W',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'person_filter = go2w_perception.person_filter:main',
            'person_follower = go2w_perception.person_follower:main',
            'fake_person = go2w_perception.fake_person:main',
        ],
    },
)
