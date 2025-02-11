from setuptools import setup

package_name = 'duckiematrix_interface'
packages = ['duckiematrix_interface']

setup(
    name=package_name,
    version='1.0.0',
    packages=packages,
    package_dir={"": "include"},
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Davide Iafrate',
    maintainer_email='davide.iafrate@duckietown.com',
    tests_require=['pytest'],
    entry_points={},
)

