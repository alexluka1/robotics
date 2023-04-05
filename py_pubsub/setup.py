from setuptools import setup

package_name = 'py_pubsub'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kasm-user',
    maintainer_email='kasm-user@todo.todo',
    description='Introduction package to ROS2. Use as ref when making new nodes.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={ # Where the publisher can be accessed
        'console_scripts': [
            'talker = py_pubsub.publisher_member_function:main', # Talker node I guess
            'listener = py_pubsub.subscriber_member_function:main',
        ],
    },
)
