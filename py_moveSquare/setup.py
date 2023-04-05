from setuptools import setup

package_name = 'py_moveSquare'

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
    description='Simple implementation to move the robot in a square.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square = py_moveSquare.pubSquare:main', 
        ],
    },
)
