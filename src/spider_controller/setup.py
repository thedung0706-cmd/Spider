import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'spider_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thedung',
    maintainer_email='thedung@todo.todo',
    description='Hexapod V3.0 Controller with Deep RL',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Nạp bộ não Học tăng cường sâu (PPO) điều khiển Mô-men xoắn
            'rl_node = spider_controller.hexapod_env:main',
            # Giữ lại module IK cũ để làm Bộ tạo chuyển động (RTMG)
            'ik_node = spider_controller.inverse_kinematics:main',
        ],
    },
)