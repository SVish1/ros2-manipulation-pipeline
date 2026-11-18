from setuptools import find_packages, setup

package_name = 'manipulation_pipeline'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description=('Fault-tolerant pick-and-place pipeline for the Franka Panda '
                 'using MoveIt 2 and TF2.'),
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scene_broadcaster = manipulation_pipeline.scene_broadcaster:main',
            'pick_place_node = manipulation_pipeline.pick_place_node:main',
        ],
    },
)
