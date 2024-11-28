from setuptools import setup

package_name = 'rccar_bringup'

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
    maintainer='dunoran',
    maintainer_email='pysqkrdytpq@snu.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'RLLAB_project1 = rccar_bringup.project.IS_RLLAB.project.RLLAB_project1:main',
            'RLLAB_project2 = rccar_bringup.project.IS_RLLAB.project.RLLAB_project2:main',
            'RLLAB_project3 = rccar_bringup.project.IS_RLLAB.project.RLLAB_project3:main'
        ],
    },
)
