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
        ('share/' + package_name + '/launch', ['launch/slam_n_nav.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/config', ['config/mapper_params_online_async.yaml']),
        ('share/' + package_name + '/launch', ['launch/exploration.launch.py']),
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
            'explorer = frontier_exploration.explorer_node:main',
            'navigation = frontier_exploration.navigation_node:main',
        ],
    },
)
