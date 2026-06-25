from setuptools import find_packages, setup

package_name = 'motor_serial_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@example.com',
    description='Serial motor driver for a custom two-wheel Yahboom motor board robot.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_serial_driver = motor_serial_driver.motor_serial_driver:main',
        ],
    },
)
