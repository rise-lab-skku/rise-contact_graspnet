import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rise_contact_graspnet2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(
        include=[
            'contact_graspnet',
            'pointnet2',
        ],
        exclude=['test'],
    ),
    data_files=[
        (
            os.path.join('share', package_name, 'contact_graspnet'),
            glob(os.path.join('contact_graspnet', '*.[py][yaml]*')),
        ),
        # (
        #     os.path.join('share', package_name, 'pointnet2'),
        #     glob(os.path.join('pointnet2', '*')),
        # ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*')),
            # glob(os.path.join('launch', '*launch.[pxy][yma]*')),
            # glob(os.path.join('launch', '*.cfg')),
        ),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sangyoon',
    maintainer_email='sangyoon@g.skku.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_node = contact_graspnet.ros2_node:main',
            'client = contact_graspnet.ros2_client_example:main',
        ],
    },
)
