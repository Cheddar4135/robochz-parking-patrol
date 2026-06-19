from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robochz_bringup'


def package_files(directory):
    """하위폴더/바이너리 구조를 보존해 data_files 튜플 리스트로 변환 (모델 디렉토리용)."""
    paths = []
    for (path, _dirs, filenames) in os.walk(directory):
        files = [os.path.join(path, f) for f in filenames]
        if files:
            paths.append((os.path.join('share', package_name, path), files))
    return paths


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
    ] + package_files('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dahyun',
    maintainer_email='dahyun@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dynamic_car_mover = robochz_bringup.dynamic_car_mover:main',  # 고정경로 동적차량
        ],
    },
)
