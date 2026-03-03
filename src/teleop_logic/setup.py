from setuptools import setup, find_packages

package_name = 'teleop_logic'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/teleop_all.launch.py']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='thesun',
    maintainer_email='thesun@teleop.dev',
    description='MG400 VR Teleop Logic',
    license='MIT',
    entry_points={
        'console_scripts': [
            'teleop_node = teleop_logic.teleop_node:main',
            'monitor_gui = teleop_logic.monitor_gui:main',
            'mock_frontend = teleop_logic.mock_frontend:main',
        ],
    },
)
