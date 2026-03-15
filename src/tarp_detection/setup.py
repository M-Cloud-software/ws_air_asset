from setuptools import setup, find_packages

package_name = 'tarp_detection'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/detection.launch.py', 'launch/sitl.launch.py']),
        ('share/' + package_name + '/config',
            ['config/detection_params.yaml', 'config/monitor_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='HSV + CCA tarp detection for Jetson Orin Nano',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tarp_detection_node = tarp_detection.tarp_detection_node:main',
            'sitl_publisher = tarp_detection.sitl_publisher:main',
            'pipeline_monitor = tarp_detection.pipeline_monitor_node:main',
        ],
    },
)
