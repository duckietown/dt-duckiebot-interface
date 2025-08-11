from setuptools import setup

package_name = 'display_driver'
packages = ["display_driver", "display_driver_node", "display_hardware_test"]

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
    maintainer='Andrea F. Daniele',
    maintainer_email='afdaniele@duckietown.com',
    tests_require=['pytest'],
    entry_points={},
)

