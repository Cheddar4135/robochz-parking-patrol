import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robochz_description'


def package_files(directory):
    """하위폴더/바이너리 구조를 보존해 data_files 튜플 리스트로 변환 (models 디렉토리용)."""
    paths = []
    for (path, _dirs, names) in os.walk(directory):
        files = [os.path.join(path, n) for n in names]
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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ] + package_files('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dahyun',
    maintainer_email='studymode030303@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
