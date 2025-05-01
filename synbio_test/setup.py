from setuptools import find_packages, setup

package_name = 'synbio_test'

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
    maintainer='case',
    maintainer_email='caseashton2@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sending_gpio = synbio_test.sending_gpio:main',
            'rs_read = synbio_test.rs_read:main',
        ],
    },
)
