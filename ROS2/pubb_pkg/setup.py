from setuptools import setup
from glob import glob
import os


package_name = 'pubb_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name),glob('launch/assem_launch.py')),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='imchin',
    maintainer_email='imarkchin@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_node = pubb_pkg.pub_node:main',
            'sub_n_pub_node = pubb_pkg.sub_n_pub_node:main',
            'tservice_node = pubb_pkg.tservice_node:main',
            'taction_node = pubb_pkg.t_action_node:main',
            'pub_custom_node = pubb_pkg.pub_custom_node:main',
            'muti_node = pubb_pkg.muti_node:main'

            
        ],
    },
)
