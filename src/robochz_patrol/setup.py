from setuptools import find_packages, setup

package_name = 'robochz_patrol'

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
    maintainer='dahyun',
    maintainer_email='dahyun@todo.todo',
    description='Controls autonomous patrol and requests plate recognition',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'patrol_node = robochz_patrol.patrol_node:main',
        ],
    },
)
