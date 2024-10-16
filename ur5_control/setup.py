from setuptools import find_packages, setup

package_name = 'ur5_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, [package_name + '/__init__.py']),
        # ('lib/' + package_name + '/pyarmor_runtime_000000', [package_name + '/pyarmor_runtime_000000/pyarmor_runtime.so']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='magic',
    maintainer_email='vimalgracerobotics@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "task1b = ur5_control.task1b_boiler_plate:main",
        ],
    },
)
