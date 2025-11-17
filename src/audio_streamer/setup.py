from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'audio_streamer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy]*'))),(os.path.join('share', package_name, 'config'), glob('config/params.yaml'))
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'sounddevice'],
    zip_safe=True,
    maintainer='mariem',
    maintainer_email='mariem@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'audio_pub = audio_streamer.audio_publisher:main',
            'audio_sub = audio_streamer.audio_subscriber:main',
        ],
    },
)
