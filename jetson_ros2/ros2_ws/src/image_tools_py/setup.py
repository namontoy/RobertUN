from setuptools import find_packages, setup

package_name = 'image_tools_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ingfisica',
    maintainer_email='aldairmolina@gmail.com',
    description='Image-first perception tools for local image testing.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'color_car_counter = image_tools_py.color_car_counter_node:main',
            'color_car_counter_service = image_tools_py.color_car_counter_service_node:main',
        ],
    },
)
