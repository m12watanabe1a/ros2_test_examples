from setuptools import setup

package_name = 'ros2_test_examples_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='m12watanabe1a',
    maintainer_email='m12watanabe1a@gmail.com',
    description='ros2 test example package',
    license='Appache License 2.0',
    tests_require=['pytest'],
    entry_points={
            'console_scripts': [
                '{node} = {pkg_path}.{filename}:{entry}'.format(
                    node='talker',
                    pkg_path=package_name,
                    filename='talker',
                    entry='main',
                ),
                '{node} = {pkg_path}.{filename}:{entry}'.format(
                    node='listener',
                    pkg_path=package_name,
                    filename='listener',
                    entry='main',
                ),
            ],
    },
)
