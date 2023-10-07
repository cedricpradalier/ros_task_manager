from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'button_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share/',package_name), ['package.xml']),
        #Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch'))),
        (os.path.join('share', package_name, 'lib'), glob(os.path.join('lib', 'jquery-3.6.4.min.js'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cedricp',
    maintainer_email='cedricp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
    scripts=[
        'nodes/button_server_node',
    ],
)
