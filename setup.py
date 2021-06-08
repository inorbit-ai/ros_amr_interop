from setuptools import setup, find_packages

package_name = 'ros2_to_mass_amr_interop'

setup(
    name=package_name,
    packages=find_packages(),
    version='0.0.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='InOrbit',
    maintainer_email='support@inorbit.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2_to_mass_amr_interop = ros2_to_mass_amr_interop.main:main'
        ],
    },
)
