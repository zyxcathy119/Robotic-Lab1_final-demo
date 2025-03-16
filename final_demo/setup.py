from setuptools import find_packages, setup

package_name = 'final_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages', ['resource/final_demo']),
    ('share/final_demo', ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zyx119',
    maintainer_email='zyxcathy119@126.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'trajectory_from_csv = final_demo.trajectory_from_csv:main',
        'plot_trajectory = final_demo.plot_trajectories:main',
        'gen3lite_pymoveit2 = final_demo.gen3lite_pymoveit2:main',
        'final_demo_arm = final_demo.final_demo_arm:main',
        'run_turtlebot = final_demo.run_turtlebot:main',
        'run_maze = final_demo.run_maze:main',
    ],
    },
)
