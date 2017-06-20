#!/usr/bin/env python

from setuptools import setup

setup(
    name='rosserial_python',
    version='0.7.6',
    packages=[],
    py_modules=[
        'nodes.message_info_service', 'nodes.serial_node',
        'src.rosserial_python.SerialClient'],
    install_requires=['setuptools'],
    author='Michael Ferguson',
    maintainer='Hunter L. Allen',
    maintainer_email='hunter@openrobotics.org',
    keyword=['ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'A Python-based implementation of the ROS serial protocol.'
    ),
    license='BSD',
    # todo: test_suite=test
    entry_points={
        'console_scripts': [
            'message_info_service = nodes.message_info_service:main',
            'serial_node = nodes.serial_node:main'
        ]
    },
)

# d = generate_distutils_setup(
#     packages=['rosserial_python'],
#     package_dir={'': 'src'},
#     )
#
# setup(**d)
