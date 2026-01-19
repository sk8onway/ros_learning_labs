from setuptools import setup

package_name = 'joke_teller'

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
    maintainer='saket',
    maintainer_email='saketsalt@gmail.com',
    description='ROS 2 learning labs',
    license='MIT',
    entry_points={
        'console_scripts': [
            'multiply_server = joke_teller.multiply_server:main',
            'multiply_server_custom = joke_teller.multiply_server_custom:main',
            'multiply_server_client = joke_teller.multiply_server_client:main'
        ],
    },
)
