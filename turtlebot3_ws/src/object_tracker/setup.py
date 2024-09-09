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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pengyuan',
    maintainer_email='pengyuan.shu@outlook.com',
    description='Used for object detection in turtlebot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'find_object    = object_tracker.find_object:main',
            'find_camera_id = object_tracker.find_camera_id:main',
            'find_color     = object_tracker.find_color:main',
            'bounding_box   = object_tracker.bounding_box:main',
            'talker         = object_tracker.publisher:main',
            'listener       = object_tracker.subscriber:main'
        ],
    },
)
