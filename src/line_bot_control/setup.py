from setuptools import find_packages, setup

package_name = 'line_bot_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vinh',
    maintainer_email='vinh@todo.todo',
    description='Line Bot Controller',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Sửa 'line_bot_bringup' thành 'line_bot_control'
            'line_node = line_bot_control.line_node:main',
            'robot_brain = line_bot_control.robot_brain:main',
            'sonar_node = line_bot_control.sonar_node:main',
            'visualize_node = line_bot_control.visualize_node:main', 
        ],
    },
)