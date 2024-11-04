from setuptools import setup
import glob
import os

package_name = 'yolo_segmentation'

model_files = glob.glob(os.path.join('model', '*'))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/yolo_segmentation_launch.py']),
        ('share/' + package_name + '/model', model_files), 
    ],
    install_requires=[
        'numpy>=1.23.5,<2.0.0',
        'opencv-python==4.8.1.78',
        'matplotlib',
        'ultralytics',
        'torch'
    ],
    zip_safe=True,
    maintainer='mjlee111',
    maintainer_email='menggu1234@naver.com',
    description='Image recognition package using Yolo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_segmentation_node = yolo_segmentation.yolo_segmentation_node:main',
        ],
    },
)
