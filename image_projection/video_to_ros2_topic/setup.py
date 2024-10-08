from setuptools import setup
import glob
import os

package_name = 'video_to_ros2_topic'

video_files = glob.glob(os.path.join('videos', '*'))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/video_to_topic.py']),
        ('share/' + package_name + '/videos', video_files)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mjlee111',
    maintainer_email='menggu1234@naver.com',
    description='Image projection package for publishing video file to topic',
    license='Apache-2.0',  
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'video_to_ros2_topic_node = video_to_ros2_topic.video_to_ros2_topic_node:main'
        ],
    },
)
