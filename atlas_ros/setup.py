from setuptools import setup
import os

package_name = 'atlas_ros'

urdf_files = []
for root, directory, file in os.walk(os.getcwd() + "/resource/atlas"):
    for f in file:
        file_rel_path = os.path.join(os.path.relpath(root, os.getcwd()), f)
        install_dir = "share/" + package_name + "/" + os.path.relpath(root, os.getcwd())
        urdf_files.append((install_dir, [file_rel_path]))

data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + urdf_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwilson',
    maintainer_email='jessemaxxwilson@utexas.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
