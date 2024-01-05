from setuptools import find_packages, setup

package_name = 'sasori'

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
    maintainer='Tiago Rainho',
    maintainer_email='tiago.rainho@ua.pt',
    description='Sasori',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot = sasori.robot_publisher:main',
            'brain = sasori.brain:main',
            'main = sasori.main:main',
            'node = sasori.robot_node:main'
        ],
    },
)
