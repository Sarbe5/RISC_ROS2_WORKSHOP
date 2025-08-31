from setuptools import find_packages, setup

package_name = 'pack'

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
    maintainer='rohit',
    maintainer_email='rohitvn1401@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "move_square = pack.move_square:main",
            "talker = pack.talker:main",
            "listner = pack.listner:main",
            "move_st = pack.move_st:main",
            "move_circle = pack.move_circle:main",
            "move_spiral = pack.move_spiral:main",
            "move_back_and_forth = pack.back_and_forth:main"
        ],
    },
)
