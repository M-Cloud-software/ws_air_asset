from setuptools import setup, find_packages

package_name = 'jetson_modem'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/modem.launch.py']),
        ('share/' + package_name + '/config',
            ['config/modem_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Cellular modem transmitter for MCLOUD drone pipeline',
    license='MIT',
    entry_points={
        'console_scripts': [
            'jetson_modem_node = jetson_modem.jetson_modem_node:main',
        ],
    },
)
