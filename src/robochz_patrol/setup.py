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
            'patrol_node = robochz_patrol.patrol_node:main',            # Nav2 버전
            'patrol_rulebased = robochz_patrol.patrol_rulebased:main',  # Rule-based 버전
            'map_odom_calib = robochz_patrol.map_odom_calib:main',      # 정적 map→odom 보정
            'path_recorder = robochz_patrol.path_recorder:main',        # 주행경로 기록/시각화
        ],
    },
)
