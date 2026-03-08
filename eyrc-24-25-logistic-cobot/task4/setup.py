from setuptools import find_packages, setup

package_name = 'task4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anees',
    maintainer_email='anisameen123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "navigation = task4.ebot_nav2_cmd:main",
            "docking = task4.ebot_docking_service:main",
            "passing = task4.ebot_passing_service:main"
        ],
    },
)
