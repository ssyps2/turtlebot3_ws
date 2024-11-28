from setuptools import find_packages, setup

package_name = 'maze_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/maze_navigation.launch.py']), 
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
            'image_recog_knn   = maze_navigation.image_recog_knn:main',
            'image_recog_svm   = maze_navigation.image_recog_svm:main',
            'getObjectRange    = maze_navigation.getObjectRange:main',
            'getObjectRange1    = maze_navigation.getObjectRange1:main',
            'goToGoal          = maze_navigation.goToGoal:main',
            'find_color        = maze_navigation.find_color:main'
        ],
    },
)
