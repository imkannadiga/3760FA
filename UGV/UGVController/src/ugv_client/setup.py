from setuptools import setup

package_name = 'ugv_client'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools', 'websockets'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    entry_points={
        'console_scripts': [
            'ugv_client = ugv_client.client:main',  # This starts the UGV controller
            'map_server_client = ugv_client.map_server_client:main' # This starts the map server client
        ],
    },
)