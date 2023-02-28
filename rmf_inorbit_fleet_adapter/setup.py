from glob import glob
import os
from setuptools import setup

package_name = 'rmf_inorbit_fleet_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name),
         glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author="Tomás Badenes tomasbadenes@gmail.com",
    maintainer='Julian Cerruti',
    maintainer_email='julian@inorbit.ai',
    description='Full Control Fleet Adapter for integrating InOrbit with Open-RMF',
    license='3-Clause BSD License',
    entry_points={
        'console_scripts': [
            'fleet_adapter=rmf_inorbit_fleet_adapter.fleet_adapter:main',
        ],
    },
)
