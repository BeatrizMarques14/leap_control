from setuptools import find_packages, setup

package_name = 'leap_hand_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','dynamixel_sdk'],
    zip_safe=True,
    maintainer='beatrix',
    maintainer_email='marques.beatriz14@ua.pt',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'read_position = leap_hand_control.individual_nodes.read_position:main',
            'read_velocity = leap_hand_control.individual_nodes.read_velocity:main',
            'read_pwm = leap_hand_control.individual_nodes.read_pwm:main',
        ],
    },
)
