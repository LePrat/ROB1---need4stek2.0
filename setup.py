'''
    LE FICHIER .REPOS DOIT IMPORTER TURTLEBOT 3, CREER DEUX DOSSIERS ET BUILD LES DEUX
'''

# Importer dépendances turtlebot3 dans .repos
# Visualiser une map avec la stat navigation sur rviz



import os
from glob import glob
from setuptools import setup

package_name = 'need4stek'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/' + package_name, ['resource/worlds/challenge_maze.world'])

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='philippe',
    maintainer_email='philippe.desousaviolante@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capability1 = need4stek.main:main',
        ],
    },
)
