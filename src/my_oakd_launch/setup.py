from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'my_oakd_launch'

data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]


def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:
        
        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)
                    
                else:
                    paths_dict[install_path] = [file_path]
                
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files


setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='src'),  # Include src/ as package directory
    package_dir={'': 'src'},
    data_files=package_files(data_files, ['launch/', 'config/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itr',
    maintainer_email='hamish.grant@tum.de',
    description='Launch OAK-D with Isaac Perceptor',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "image_converter = my_oakd_launch.image_converter:main",  # Reference src/image_converter.py
        ],
    },
)
