# launch
import os

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Obtener rutas a los archivos de configuración
    package_dir = get_package_share_directory('ex_recu_24_preg_02') #cambiar
    nav2_yaml = os.path.join(package_dir, 'config', 'my_nav2_params_full.yaml') #cambiar
    map_file = os.path.join(package_dir, 'config', 'office_map.yaml') # cambiar
    rviz_config_dir = os.path.join(package_dir, 'config', 'my_slam.rviz') #cambiar
    
    # Verificar si los archivos existen y mostrar rutas -- cambiar
    print(f"Archivo de navegación: {nav2_yaml} - Existe: {os.path.exists(nav2_yaml)}")
    print(f"Archivo de mapa: {map_file} - Existe: {os.path.exists(map_file)}")
    print(f"Archivo de RViz: {rviz_config_dir} - Existe: {os.path.exists(rviz_config_dir)}")

    return LaunchDescription([
        # Servidor de mapas
        Node(
            package = 'nav2_map_server',
            executable = 'map_server',
            name = 'map_server',
            output = 'screen',
            parameters=[
                {'use_sim_time': True},
                {'yaml_filename': map_file}
            ]
        ),

        # Localización AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[
                nav2_yaml,
                {'use_sim_time': True}
            ]
        ),

        # Lifecycle Manager para localización
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': ['map_server', 'amcl']}
            ]
        ),

        # Visualizador RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': True}],
            output='screen'
        )
    ])