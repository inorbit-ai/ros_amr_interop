from setuptools import setup, find_packages

package_name = 'massrobotics_amr_sender'

setup(
    name=package_name,
    packages=find_packages(),
    package_data={'': ['schema.json']},
    include_package_data=True,
    version='0.0.2',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='InOrbit',
    maintainer_email='support@inorbit.ai',
    description='ROS2 node implementing a MassRobotics AMR Interoperability Sender',
    license='3-Clause BSD License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'massrobotics_amr_node = massrobotics_amr_sender.massrobotics_amr_node:main'
        ],
    },
)
