from setuptools import find_packages, setup

package_name = 'my_package_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='prasun',
    maintainer_email='prasun.ieee@gmail.com',
    description='Beginner client libraries tutorials practice package.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node_python = my_package_python.my_node_python:main'
        ],
    },
)
