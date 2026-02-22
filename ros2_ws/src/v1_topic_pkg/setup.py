from setuptools import find_packages, setup

package_name = 'v1_topic_pkg'

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
    maintainer='bachelor',
    maintainer_email='bachelor@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'laser_data_publisher = v1_topic_pkg.laser_data_publisher:main',
        'laser_data_subscriber= v1_topic_pkg.laser_data_subscriber:main',
        'set_scan_frequency_client= v1_topic_pkg.set_scan_frequency_client:main'
        ],
    },
)
