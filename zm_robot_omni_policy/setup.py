from glob import glob
from setuptools import setup

package_name = 'zm_robot_omni_policy'

# The models/ directory (ONNX, TensorRT engines, checkpoints: > 1 GB) is deliberately NOT
# installed; the nodes find it through the model_dir parameter, $ZM_ROBOT_OMNI_MODEL_DIR or
# the source tree (see zm_robot_omni_policy/paths.py).
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    extras_require={
        'test': ['pytest'],
    },
    zip_safe=True,
    maintainer='zmtech',
    maintainer_email='qaz9517532846@gmail.com',
    description='Omnidirectional residual-SAC + X-Mobility navigation policy for zm_robot.',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'omni_policy_node = zm_robot_omni_policy.omni_policy_node:main',
            'replay_validation_node = zm_robot_omni_policy.replay_validation_node:main',
        ],
    },
)
