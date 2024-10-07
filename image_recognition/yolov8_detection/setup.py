from setuptools import setup
import glob
import os

package_name = 'yolov8_detection'

model_files = glob.glob(os.path.join('model', '*'))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolov8_detection_launch.py']),
        ('share/' + package_name + '/model', model_files), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='m',
    maintainer_email='menggu1234@naver.com',
    description='Image recognition package using YoloV8',
    license='Apache2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov8_detection_node = yolov8_detection.yolov8_detection_node:main',
        ],
    },
)
