#nameOfLaunchFile.launch.py
# Import the necessary libraries
from launch import LaunchDescription  # Core library for defining a launch file in ROS 2
from launch_ros.actions import Node  # To define and launch ROS 2 nodes
from launch_ros.parameter_descriptions import ParameterValue  # To pass parameters dynamically to nodes
from launch.actions import DeclareLaunchArgument  # To define launch arguments for dynamic configuration
from launch.substitutions import Command, LaunchConfiguration  # To handle runtime substitutions
import os  # Python's standard library for file path manipulations
from ament_index_python import get_package_share_directory  # To find package resources in ROS 2

# Define the launch file description
def generate_launch_description():
    # Declare a launch argument for the robot's model file (Xacro/URDF)
    # Uses DeclareLaunchArgument from launch.actions
    model_arg = DeclareLaunchArgument(
        name="model",  # Name of the argument
        default_value=os.path.join(
            get_package_share_directory("arduinobot_description"),  # Locate the package directory (ament_index_python)
            "urdf",  # Subdirectory for URDF/Xacro files
            "arduinobot.urdf.xacro"  # Default file name
        ),
        description="Absolute path to the robot URDF file"  # Explain the purpose of this argument
    )
    
    # Define the robot description parameter
    # Uses Command (from launch.substitutions) to process the Xacro file into a URDF
    # Uses LaunchConfiguration (from launch.substitutions) to retrieve the user-specified "model" argument at runtime
    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")])) #space after xacro

    # Define the robot_state_publisher node
    # Uses Node (from launch_ros.actions) to create and configure the node
    robot_state_publisher = Node(
        package="robot_state_publisher",  # The ROS 2 package containing the node
        executable="robot_state_publisher",  # The node executable
        parameters=[{"robot_description": robot_description}]  # Pass the processed URDF as a parameter
    )

    # Define the joint_state_publisher_gui node
    # Uses Node (from launch_ros.actions) to create and configure the node
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",  # The ROS 2 package for the joint state GUI
        executable="joint_state_publisher_gui"  # The node executable for the GUI
    )

    # Define the RViz2 visualization node
    # Uses Node (from launch_ros.actions) to create and configure the node
    rviz_node = Node(
        package="rviz2",  # The ROS 2 package containing RViz2
        executable="rviz2",  # The node executable for RViz2
        name="rviz2",  # Node name in the ROS graph
        arguments=["-d", os.path.join(
            get_package_share_directory("arduinobot_description"),  # Locate the package directory (ament_index_python)
            "rviz",  # Subdirectory for RViz configuration files
            "display.rviz"  # The specific RViz configuration file to load
        )]
    )

    # Return the full launch description
    # Uses LaunchDescription (from launch) to group all actions together
    return LaunchDescription([
        model_arg,  # Include the model argument declaration
        robot_state_publisher,  # Include the robot_state_publisher node
        joint_state_publisher_gui,  # Include the joint_state_publisher_gui node
        rviz_node  # Include the RViz2 node
    ])
