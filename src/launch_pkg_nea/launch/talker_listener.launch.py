#librerias python para la ejecución de multiples nodos

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Definicion del nodo (pkg_name, node_name)
    talker = Node(
            package='demo_nodes_cpp',
            executable='talker'
            )

    # Definicion del nodo (pkg_name, node_name)
    listener = Node(
            package='demo_nodes_cpp',
            executable='listener'
            )

    #retorno "generate_launch_description", separar por comas, el objeto de cada nodo
    return LaunchDescription([
        talker,
        listener
    ])