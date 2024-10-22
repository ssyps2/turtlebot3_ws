from setuptools import find_packages, setup

package_name = 'navi2goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/avoid_obstacles.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pengyuan',
    maintainer_email='pengyuan.shu@outlook.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'getObjectRange = navi2goal.getObjectRange:main',
        'goToGoal = navi2goal.goToGoal:main'
        ],
    },
)
