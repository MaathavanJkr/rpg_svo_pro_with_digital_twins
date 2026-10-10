from setuptools import setup

package_name = 'vikit_py'

setup(
    name=package_name,
    version='0.0.0',
    package_dir={package_name: 'src/vikit_py'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christian Forster',
    maintainer_email='forster@ifi.uzh.ch',
    description='Python utilities of vikit (trajectory alignment, ROS 2 bag helpers, ...)',
    license='BSD',
)
