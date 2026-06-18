from setuptools import find_packages, setup

package_name = 'robochz_perception'

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
    description='Performs plate recognition and publishes detection results',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'perception_node = robochz_perception.perception_node:main',
        ],
    },
)
