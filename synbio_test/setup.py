from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'synbio_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='case',
    maintainer_email='caseashton2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sending_gpio = synbio_test.sending_gpio:main',
            'depth_features = synbio_test.depth_features:main',
            'fixed_frame_broadcaster = synbio_test.fixed_frame_broadcaster:main',
            'tf_listener = synbio_test.tf2_listener:main',
            'rgb_features = synbio_test.rgb_features:main'
        ],
    },
)
