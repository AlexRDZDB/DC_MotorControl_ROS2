from setuptools import setup

package_name = 'input_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # Asegura que el paquete se registre correctamente
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alfonso',
    maintainer_email='alfonso@todo.todo',
    description='Input Node for Micro-ROS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'input_node = input_node.input_node:main',  # Asegurar coincidencia con input_node.py
        ],
    },
)
