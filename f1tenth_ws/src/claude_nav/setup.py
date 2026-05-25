from setuptools import setup

package_name = 'claude_nav'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Claude AI-driven camera navigation for F1Tenth',
    license='MIT',
    entry_points={
        'console_scripts': [
            'claude_nav = claude_nav.claude_nav_node:main',
        ],
    },
)
