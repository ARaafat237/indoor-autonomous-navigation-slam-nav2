from setuptools import find_packages, setup

package_name = 'differential_odom'

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
    description='Differential-drive odometry integration from raw wheel velocity.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'differential_odom_node = differential_odom.differential_odom_node:main',
        ],
    },
)
