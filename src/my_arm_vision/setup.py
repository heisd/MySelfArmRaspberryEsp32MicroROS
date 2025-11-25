from setuptools import setup

package_name = 'my_arm_vision'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='heisd',
    maintainer_email='2284610019@qq.com',
    description='视觉抓取包',
    license='MIT',
    entry_points={
        'console_scripts': [
            'object_detector = my_arm_vision.object_detector:main',
            'yolo_detector = my_arm_vision.yolo_detector:main',
            'hand_eye_calibration = my_arm_vision.hand_eye_calibration:main',
            'visual_grasp_controller = my_arm_vision.visual_grasp_controller:main',
            'simple_visual_grasp = my_arm_vision.simple_visual_grasp:main',
        ],
    },
)