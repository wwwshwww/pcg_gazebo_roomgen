import os
import sys
import setuptools
import codecs

version_file = os.path.join(os.path.dirname(__file__), 'roomor/version.py')

with open(version_file, 'r') as f:
    __version__ = eval(f.read().strip().split('=')[-1])
    
with codecs.open("README.md", "r", encoding="utf-8") as fh:
    long_description = fh.read()
    
requirements = [
    'numpy-quaternion',
    'trimesh[easy]',
    'shapely',
    'scikit-learn',
    'pcg_gazebo'
]

setuptools.setup(
    name='roomor',
    version=__version__,
    description='A Python framework for generating and deploying random room for Gazebo',
    long_description=long_description,
    long_description_content_type='text/markdown',
    author='Reona Sato',
    author_email='www.shinderu.www@gmail.com',
    url='https://github.com/wwwshwww/roomor',
    packages=setuptools.find_packages(),
    classifiers=[
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3.6',
    ],
    keywords='gazebo ros simulation robotics autogeneration environment reinforcement-learning pcg_gazebo',
    install_requires=requirements
)