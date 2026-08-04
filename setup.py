from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'valm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py'),),
        
        #path to world folder
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sam',
    maintainer_email='sam@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = valm.controller:main',
        ],
    },
)
