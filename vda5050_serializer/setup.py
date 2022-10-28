from setuptools import setup

package_name = 'vda5050_serializer'

setup(
    name=package_name,
    version='1.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pytest'],
    zip_safe=True,
    maintainer='Jannik Abbenseth',
    maintainer_email='jannik.abbenseth@ipa.fraunhofer.de',
    description='The serialization/deserialization functions to be loaded into ' +
    'the mqtt_bridge. During(de-)serialization the case of the keys is corrected' +
    ' from snake_case on ROS side to dromedaryCase on MQTT/JSON side ',
    license='Apache2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
    python_requires='>=3.5'
)
