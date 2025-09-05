from setuptools import find_packages, setup

package_name = 'inspire_retargeting'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'handpoints_publisher',
        ],
    zip_safe=True,
    maintainer='zz',
    maintainer_email='zz@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "detect_from_csv = inspire_retargeting.detect_from_csv:main",
        'pub_csv = inspire_retargeting.pub_csv:main',
        ],
    },
)
