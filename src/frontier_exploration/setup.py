from setuptools import find_packages, setup

package_name = 'frontier_exploration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/full_simulation.launch.py',
        ]),
    ],
    install_requires=['setuptools',  'numpy'],
    zip_safe=True,
    maintainer='seabass',
    maintainer_email='sebastian.herrera.t@ug.uchile.cl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = frontier_exploration.detector:main',
            'wait_for_clock = frontier_exploration.wait_for_clock:main',
            'odom_tf_publisher = frontier_exploration.odom_tf_publisher:main',
        ],
    },
)
