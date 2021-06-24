from setuptools import setup, find_packages

package_name = 'mrai_sender'

setup(
    name=package_name,
    packages=find_packages(),
    package_data={'': ['schema.json']},
    include_package_data=True,
    version='0.1.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='InOrbit',
    maintainer_email='support@inorbit.ai',
    description='ROS2 node implementing MassRobotics AMR Interoperability Standard',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mrai_node = mrai_sender.mrai_node:main'
        ],
    },
)
