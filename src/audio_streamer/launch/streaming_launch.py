import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 1. Trova il nome del pacchetto e la sua directory condivisa
    package_name = 'audio_streamer'
    pkg_share_dir = get_package_share_directory(package_name)

    # 2. Definisci il percorso completo del file di parametri YAML
    params_file = os.path.join(pkg_share_dir, 'config', 'params.yaml')

    # 3. Definizione del nodo Publisher
    # L'eseguibile 'audio_pub' è quello definito nel  setup.py
    audio_publisher_node = Node(
        package=package_name,
        executable='audio_pub',
        name='audio_publisher',
        output='screen',
        # Carica i parametri definiti nel file params.yaml
        parameters=[params_file],
    )

    # 4. Definizione del nodo Subscriber
    audio_subscriber_node = Node(
        package=package_name,
        executable='audio_sub',
        name='audio_subscriber',
        output='screen',
        # Carica i parametri definiti nel file params.yaml
        parameters=[params_file],
    )

    # 5. Restituisce la descrizione di lancio che include entrambi i nodi
    return LaunchDescription([
        audio_publisher_node,
        audio_subscriber_node,
    ])
