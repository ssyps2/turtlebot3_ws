from setuptools import find_packages, setup

package_name = 'camera_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_recognition.launch.py']),  # Include launch files  !!!Pay Attention to this
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruize',
    maintainer_email='crzrizer@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'client_test = camera_recognition.client_test:main',
             'color_track_server = camera_recognition.color_track_server:main',
        ],
    },
)
