from setuptools import find_packages, setup

package_name = 'mrs_project'

setup(
    name=package_name,
    version='0.0.0',
    # Automatically find all packages except 'test'
    packages=find_packages(where='src', exclude=['test']),
    
    # Installing the necessary data files for the package
    data_files=[
        # Register the package under the ROS 2 index
        ('share/ament_index/resource_index/packages', 
         ['resource/' + package_name]),

        # Install the package.xml to the package directory
        ('share/' + package_name, ['package.xml']),

        # Optionally add the launch files and other necessary files
        ('share/' + package_name + '/launch', ['launch/reynolds.launch.py']),
    ],
    
    install_requires=['setuptools'],
    zip_safe=True,

    # Package maintainer information
    maintainer='root',
    maintainer_email='renatto.tommasi@gmail.com',

    # Package description (update this with more details)
    description='This package implements the Reynolds Boids simulation for ROS 2.',

    # License
    license='Apache-2.0',

    # Test dependencies
    tests_require=['pytest'],

    # Define console scripts (entry points)
    entry_points={
        'console_scripts': [
            'reynolds = mrs_project.reynolds:main'  # Make sure `mrs_project.reynolds.main` exists
        ],
    },
)