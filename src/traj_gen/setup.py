from setuptools import find_packages, setup

package_name = 'traj_gen'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to copy the CSV:
        ('share/' + package_name + '/data', ['traj_gen/spiral_pwm_commands.csv']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sachin1299',
    maintainer_email='sachin1299@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': ['pwm8node = traj_gen.pwm8node:main',
                            'leo_debug = traj_gen.leo_debug:main',
                            'pwm_cmd = traj_gen.pwm_cmd:main'
        ],
    },
)
