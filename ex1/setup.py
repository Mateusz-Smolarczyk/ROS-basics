from setuptools import setup

package_name = 'ex1'

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
    maintainer='student_rm06',
    maintainer_email='student_rm06@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'data_publisher = ex1.data_publisher:main',
                'from_a_to_b = ex1.from_a_to_b:main',
                'trajectory = ex1.trajectory:main',
                'odom_subscriber = ex1.odom_subscriber:main'
        ],
    },
)
