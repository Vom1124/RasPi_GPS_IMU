from setuptools import setup

package_name = 'imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='vomsheendhur.raju@ndus.edu',
    description='IMU Data',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_read = imu.imu_pub_w_VnSensor:main',
            'imu_test = imu.imu_pub_wo_VnSensor:main',
        ],
    },
)
