from setuptools import find_packages, setup

package_name = 'object_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/chase_object.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pengyuan',
    maintainer_email='pengyuan.shu@outlook.com',
    description='Used for object detection and chasing in turtlebot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'find_color     = object_tracker.find_color:main',
            'detect_object  = object_tracker.detect_object:main',
            'get_object_range=object_tracker.get_object_range:main',
            'chase_object   = object_tracker.chase_object:main'
        ],
    },
)
