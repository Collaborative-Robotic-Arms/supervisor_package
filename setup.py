from setuptools import setup

package_name = 'supervisor_package'

setup(
    name=package_name,
    version='0.0.0',
    # FIXED: Changed from 'supervisor_logic' to 'supervisor_package' 
    # to match your actual folder name on disk
    packages=[package_name], 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marwan',
    maintainer_email='2100771@eng.asu.edu.eg',
    description='Supervisor node for ABB and AR4 control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # Points to main() in supervisor.py
            'supervisor = supervisor_package.supervisor:main',
            # Points to main() in test_grippers.py
            'test_grippers = supervisor_package.test_grippers:main', 
        ],
    },
)