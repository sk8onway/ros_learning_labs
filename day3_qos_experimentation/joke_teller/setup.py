from setuptools import find_packages, setup

package_name = 'joke_teller'

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
    maintainer='saket',
    maintainer_email='saketsalt@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joke_publisher = joke_teller.joke_publisher:main',
            'joke_subscriber = joke_teller.joke_subscriber:main',
            'qos_joke_publisher = joke_teller.qos_joke_publisher:main'
        ],
    },
)
