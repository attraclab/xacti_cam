from launch_ros.actions import Node
from launch.launch_context import LaunchContext
from launch.events.process.process_exited import ProcessExited
from launch.actions import RegisterEventHandler
from launch import LaunchDescription
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression

local_play = False # True to play on screen, False to not display

def node_description(local_play_flag):
	
	parameters=[{'local_play': local_play_flag}]

	return Node(
		package='xacti_cam',
		executable='camera_control',
		name='camera_control',
		parameters=parameters
	)

def on_exit_restart(event:ProcessExited, context:LaunchContext):
	print("\n\nProcess [{}] exited, pid: {}, return code: {}\n\n".format(event.action.name, event.pid, event.returncode))
	# if event.returncode != 0 and 'controller' in event.action.name:
	return node_description(local_play) # respawn node action


def generate_launch_description():

	ld = LaunchDescription([
		node_description(local_play),
		RegisterEventHandler(event_handler=OnProcessExit(on_exit=on_exit_restart))
	])

	return ld