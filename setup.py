from setuptools import find_packages, setup

package_name = 'ultrasonic_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/ultra.rviz']),
        ('share/' + package_name + '/launch', ['launch/ultrasonic.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='edwin',
    maintainer_email='edwin_george@alphadroid.io',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ultrasonic_bridge = ultrasonic_bridge.ultrasonic_bridge:main'
        ],
    },
)
