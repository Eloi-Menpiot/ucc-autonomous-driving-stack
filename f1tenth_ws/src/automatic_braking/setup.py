from distutils.core import setup

package_name = 'automatic_braking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[
        'automatic_braking.automatic_braking'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Automatic Emergency Braking for F1TENTH',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'automatic_braking = automatic_braking.automatic_braking:main'
        ],
    },
)
