from setuptools import find_packages, setup

package_name = 'image_processing_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
   
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/image_processing_pkg']),
        ('share/image_processing_pkg', ['package.xml']),
        ('share/image_processing_pkg/launch', ['launch/detection_pipeline_launch.py']),
    ],
    install_requires=['setuptools', 'rosidl_default_generators'],
    zip_safe=True,
    maintainer='turtle',
    maintainer_email='adefr011@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'image_node = image_processing_pkg.image_node:main',
        'planner_node = image_processing_pkg.planner_node:main',
    ],
},
)

# install_requires=['setuptools', 'rosidl_default_generators'],

