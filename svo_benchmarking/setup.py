from setuptools import setup

package_name = 'svo_benchmarking'

setup(
    name=package_name,
    version='0.0.0',
    package_dir={package_name: 'src/svo_ceres_benchmarking'},
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zichao Zhang',
    maintainer_email='zzhang@ifi.uzh.ch',
    description='svo_benchmarking utils',
    license='BSD',
)
