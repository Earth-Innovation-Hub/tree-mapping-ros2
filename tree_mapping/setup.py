from setuptools import setup, find_packages

package_name = 'tree_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DeepGIS Team',
    maintainer_email='deepgis@example.com',
    description='ROS 2 port of DREAMS-lab tree_mapping (offboard hover demo).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_control = tree_mapping.offboard_control:main',
        ],
    },
)
