from glob import glob
from setuptools import find_packages, setup
import os


package_name = 'tams_apriltags'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        # Scripts
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*')),
        # URDF and xacro files
        (os.path.join('share', package_name, 'urdf'), glob('models/urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('models/meshes/*')),
    ],


    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vladuinbuntu',
    maintainer_email='vladuinbuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_huge_joint_space_jump = tams_apriltags.test_huge_joint_space_jump:main',
            'execute_traj = tams_apriltags.execute_traj:main',
            'apply_collision_objects = tams_apriltags.apply_collison_objects:main',
            'move_to_cube = tams_apriltags.pick_place:main',
        ],
    },
)
