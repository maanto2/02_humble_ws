from setuptools import find_packages, setup

package_name = 'bumper_bot_py'

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
    maintainer='robinhood',
    maintainer_email='info.martinanto@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            " simple_parameter_node = bumper_bot_py.simple_parameter:main",
            " simple_turtlesim_kinematics_node = bumper_bot_py.simple_turtlesim_kinematics:main",
        ],
    },
)
