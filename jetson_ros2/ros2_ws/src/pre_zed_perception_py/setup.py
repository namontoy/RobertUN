import os

from setuptools import find_packages, setup


package_name = 'pre_zed_perception_py'
delivery_config = os.path.join(
    '..', '..', '..', 'final_preZED', 'configs', 'pre_zed_params.yaml'
)


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join('share', package_name, 'launch'),
            ['launch/pre_zed_pipeline.launch.py'],
        ),
        (
            os.path.join('share', package_name, 'config'),
            [delivery_config],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ingfisica',
    maintainer_email='aldairmolina@gmail.com',
    description='Pre-ZED local-video perception nodes for ROS 2.',
    license='MIT',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'video_file_publisher = '
            'pre_zed_perception_py.video_file_publisher:main',
            'depth_anything_http_node = '
            'pre_zed_perception_py.depth_anything_http_node:main',
            'yolo_http_detector_node = '
            'pre_zed_perception_py.yolo_http_detector_node:main',
            'overlay_node = pre_zed_perception_py.overlay_node:main',
        ],
    },
)
