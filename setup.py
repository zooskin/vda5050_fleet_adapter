from setuptools import find_packages, setup

package_name = 'vda5050_fleet_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['vda5050_fleet_adapter/config/default_params.yaml']),
    ],
    install_requires=[
        'setuptools',
        'paho-mqtt>=2.0',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='hansoo',
    maintainer_email='hansoo@todo.todo',
    description='VDA5050 Fleet Adapter for Open-RMF',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fleet_adapter = vda5050_fleet_adapter.presentation.main:main',
        ],
    },
)
