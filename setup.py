from setuptools import setup

package_name = 'gelsight_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' + package_name+'/launch', ['launch/gelsight_showimage.launch.py']),
        ('lib/python3.10/site-packages/'+package_name, ['resource/' + 'nnmini.pt'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lzh',
    description='publish gelsight image to ros2 topic',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gelsight_showimage = gelsight_ros2.gelsight_showimage:main',
            'gelsight_show3d = gelsight_ros2.gelsight_show3d:main'
        ],
    },
)
