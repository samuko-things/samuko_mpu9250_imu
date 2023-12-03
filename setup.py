from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'samuko_mpu9250_imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('lib/' + package_name, [package_name+'/modules/samuko_mpu9250_imu_serial_comm_lib.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuko',
    maintainer_email='samuel.c.agba@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'executable_name = package_name.python_file_name:main'
            'samuko_mpu9250_imu = samuko_mpu9250_imu.samuko_mpu9250_imu:main',
            'tf_test = samuko_mpu9250_imu.tf_test:main',
            # 'samuko_mpu9250_imu_serial_comm_lib = samuko_mpu9250_imu_serial_comm_lib.samuko_mpu9250_imu_serial_comm_lib:main',
        ],
    },
)

