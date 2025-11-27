from setuptools import setup

package_name = 'my_arm_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # 添加这两行来消除警告
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@email.com',
    description='视觉抓取包',
    license='MIT',
    entry_points={
        'console_scripts': [
            'object_detector = my_arm_vision.object_detector:main',
            'simple_visual_grasp = my_arm_vision.simple_visual_grasp:main',
        ],
    },
)