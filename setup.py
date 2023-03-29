from setuptools import setup

package_name = 'acquisition'

setup(
    name=package_name,
    version='0.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NotBlackMagic',
    maintainer_email='social@notblackmagic.com',
    description='Acquisition controller for AutoSQA. Gathers sensor data and position information and send them to UDP server.',
    license='BSD 3-Clause License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'acq_hub = acquisition.acquisition_hub:main',
        ],
    },
)
