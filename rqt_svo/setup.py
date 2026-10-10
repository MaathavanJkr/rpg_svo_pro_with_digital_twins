from setuptools import setup

package_name = 'rqt_svo'

setup(
    name=package_name,
    version='0.0.0',
    package_dir={'': 'src'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'plugin.xml']),
        ('share/' + package_name + '/resource', ['resource/widget.ui']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cforster',
    maintainer_email='cforster@todo.todo',
    description='rqt plugin for SVO - Semi-Direct Visual Odometry',
    license='TODO',
    entry_points={
        'console_scripts': [
            'rqt_svo = rqt_svo.main:main',
        ],
    },
)
