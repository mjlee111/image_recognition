from setuptools import setup
import glob
import os

package_name = 'image_to_ros2_topic'

image_files = glob.glob(os.path.join('images', '*'))

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/image_to_topic.py']),
        ('share/' + package_name + '/images', image_files)
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mjlee111',
    maintainer_email='menggu1234@naver.com',
    description='Image projection package for publishing image file to topic',
    license='Apache-2.0',  
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_to_ros2_topic_node = image_to_ros2_topic.image_to_ros2_topic_node:main'
        ],
    },
)
