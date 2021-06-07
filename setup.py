from setuptools import setup, find_packages

package_name = 'ros2mass'

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
    maintainer='leandro',
    maintainer_email='leandro@inorbit.ai',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros2mass = ros2mass.main:main'
        ],
    },
)
