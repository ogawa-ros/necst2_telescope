from setuptools import setup

package_name = 'necst2_telescope'

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
    maintainer='telescopio',
    maintainer_email='s_ta.matsumoto@p.s.osakafu-u.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':[
            'limit_1st='+package_name+'.soft_limit_1st:main',
            'limit_2nd='+package_name+'.soft_limit_2nd:main',
            'az_pid='+package_name+'.antenna_az_commander_pid:main',
            'el_pid='+package_name+'.antenna_el_commander_pid:main',
            'tpro01='+package_name+'.tpro01:main'
            ],
    },
)
