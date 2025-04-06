from setuptools import setup
import os #incluir
from glob import glob #incluir

package_name = 'ex_recu_24_preg_02' #cambiar

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), #cambiar
        (os.path.join('share', package_name, 'config'), glob('config/*.png')), #cambiar
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')), #cambiar
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), #cambiar
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='irene',
    maintainer_email='imedgar@epsg.upv.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
