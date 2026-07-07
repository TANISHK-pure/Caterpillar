import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('cat_paddock_sim')
    world  = os.path.join(pkg, 'worlds', 'paddock.sdf')
    bridge = os.path.join(pkg, 'config', 'bridge.yaml')
    models = os.path.join(pkg, 'models')

    set_res = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    gz = ExecuteProcess(
        cmd=['gz', 'sim', '-v', '4', '-r', world], output='screen')

    bridge_node = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge}], output='screen')

    def spawn(name, sdf_subdir, x, y, delay):
        return TimerAction(period=delay, actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                 '-world', 'cat_paddock',
                 '-file', os.path.join(models, sdf_subdir, 'model.sdf'),
                 '-name', name,
                 '-x', str(x), '-y', str(y), '-z', '0.5'],
            output='screen')])

    # Bays are now at y = -50, -75, -100  (25m vertical spacing)
    s1 = spawn('truck_1', 'truck_100t', -30, -50,  4.0)
    s2 = spawn('truck_2', 'truck_250t', -30, -75,  5.5)
    s3 = spawn('truck_3', 'truck_400t', -30, -100, 7.0)

    orchestrator = TimerAction(period=10.0, actions=[Node(
        package='cat_paddock_sim', executable='truck_orchestrator',
        output='screen')])
    perception_1 = TimerAction(period=9.0, actions=[Node(
        package='cat_paddock_sim', executable='lidar_perception',
        name='perception_1',
        parameters=[{'truck_id': 1, 'lidar_height': 6.5}],
        output='screen')])
    # hybrid_astar_node is NOT launched: all trucks are driven by
    # truck_orchestrator's ROUTES/drive_toward, not Hybrid A*. The node
    # publishes directly to /truck_1/cmd_vel (same topic the orchestrator
    # uses) the moment anything publishes to /truck_1/goal, which would
    # hijack truck_1 out from under the orchestrator. Left in the package
    # for standalone/manual use, just not auto-started here.

    return LaunchDescription([set_res, gz, bridge_node, s1, s2, s3, orchestrator, perception_1])
