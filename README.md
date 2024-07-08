# ROS2 commands and notes

## Install latest LTS version

1. Set locale <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#set-locale>
2. Setup sources <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#setup-sources>
3. sudo apt update; sudo apt upgrade
4. sudo apt install ros-humble-desktop
5. source the setup script adding this line to the end of your ~/.bashrc  
  `source /opt/ros/humble/setup.bash`
6. Try the installation running default examples:
  `ros2 run demo_nodes_cpp talker`
  `ros2 run demo_nodes_cpp listener`

## Install colcon

1. `sudo apt install python3-colcon-common-extension`
2. source the setup script to autocomplete, adding this line to the end of your ~/.bashrc
  `source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash`

## Create your project

1. Create the project directory. The convention is the following:
  `mkdir ros2_ws`
2. Create the src directory in the project dir.
  `cd ros2_ws; mkdir src`
3. Build the project
  `colcon build`
4. source the setup script of the project adding this line to the end of your ~/.bashrc
  `source /media/sf_github-sync/ROS/ros2_ws/install/setup.bash`

## Create a python package

1. Go to src of your project
  `cd src`
2. Create the package using ros command
  `ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy`
3. To compile step in the project root directory and execute:
  `cd ../`
  `colcon build`
  or you can build an specific package:
  `colcon build --packages-select my_py_pkg`

## Create a CPP package

1. Go to src of your project
  `cd src`
2. Create the package using ros command
  `ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp`
3. To compile step in the project root directory and execute:
  `cd ../`
  `colcon build`
  or you can build an specific package:
  `colcon build --packages-select my_cpp_pkg`

### Clangd setup

In order to use Clangd as language server instead of VScode Intellisense, do the following:

1. Install the Clangd extension in VSCode. If you don't have clangd installed in your OS it will
suggest the installation.

2. Open your settings.json: 
  `ctrl + shift + P` > Preferences: Open User Settings (JSON)
  Or if you are using ssh-server for remote IDE
  `ctrl + shift + P` > Preferences: Open Remote Settings (JSON)

3. Add the following configuration:
```
"clangd.arguments": [
    "-log=verbose",
    "-pretty",
    "--background-index",
    "--query-driver=/**/*",
    "--compile-commands-dir=${workspaceFolder}/build"
  ],
```
4. Add the flag `CMAKE_EXPORT_COMPILE_COMMANDS` to your CMakeLists.txt in order to generate the
compile_commands.json file required by clangd
```
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)
```
NOTE: Here you must have the executables added (add_executable and install) to your CMakeLists.

5. Build the package: `colcon build --packages-select my_cpp_pkg`

6. Restart you language server:
`ctrl + shift + P` > clangd: Restart language server

## Nodes

### Create executable to be installed (Python) 

1. Open the setup.py file of the package
2. Add to the 'console-scripts' the name of the executable and the entry point.
  `NAME_OF_EXEC = NAME_OF_PKG.NAME_OF_NODE_FILE:ENTRY_FUNCTION`
  Ex.
  `py_node = my_py_pkg.my_first_node:main`

This will create the output file in the path specified in the setup.cfg file. In this case it will be: 
ros2_ws/install/my_py_pkg/lib/my_py_pkg/py_node

3. Now you can build the pkg with: `colcon build --packages-select my_py_pkg`

NOTE: It is possible to symlink the executable in order to avoid recompilation after changes.
The python file must have execution permits.

  3.1 `chmod +x NODE_FILE_NAME.py`

  3.2 `colcon build --packages-select my_py_pkg --symlink-install`

### Create executable to be installed (Cpp)

Check the CMakeLists.txt file of my_cpp_pkg. It includes the `add_executables` and `install` macros.

### Run an installed node

ros2 run PACKAGE_NAME NODE_EXECUTABLE_NAME
Ex.
`ros2 run my_py_pkg py_node`

### List and info running nodes
`ros2 node list` and `ros2 node info NODE_NAME`

NOTE: This only give information for running nodes.

### Launch same node multiple times

`ros2 run my_py_pkg py_node --ros-args -r __node:=node2`

## Common ROS Python Node Functions

### crate_timer(period, callback)

This will create a timer where a callback will be called every given period.

## Common ROS CPP Node Functions

### create_wall_timer(period, callback)

This will create a timer where a callback will be called every given period.

## Interfaces

For available built-in-types check the [documentation](https://docs.ros.org/en/rolling/Concepts/Basic/About-Interfaces.html)

- List all installed interfaces:
`ros2 interface list`

- Get information about specific interface:
`ros2 interface show example_interfaces/msg/String`

For using already existing interfaces it is useful to use the example_interfaces provided by the lib.

In this case we want to use a msg type for a new publisher.
```
ros2 interface show example_interfaces/msg/String 
```

In the case of python you need to add the dependency to package.xml
```
<depend>example_interfaces</depend>
```

### Creating your custom interfaces

1. It's a good practice to create the custom interfaces in a dedicated package
```
ros2 pkg create my_robot_interfaces
```
2. Remove src and include directories from created package.
3. Create an msg folder in the package
```
mkdir msg
```
4. Add this 3 lines to the package.xml
```
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

5. Add the find package to CMakeLists
```
find_package(rosidl_default_generators REQUIRED)
```
6. The file name needs to be in PascalCase and have msg extension. Ex. `HardwareStatus.msg`
7. Add the types to your msg.
8. Add functions to generate the interfaces and to export the dependencies in the CMakeLists
```
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
)

ament_export_dependencies(rosidl_default_runtime)
```
9. Built the interfaces package and you will get the interfaces for python and C++.

## Topics

List the topics:
`ros2 topic list`

Print what the topic is receiving like a subscriber:
`ros2 topic echo \node-name`

Get info about the topic:
`ros2 topic info /topic_name`

Get running topic frequency:
`ros2 topic hz /topic_name`

Get running topic band width:
`ros2 topic bw /topic_name`

Publish directly to a topic:
`ros2 topic pub -r 100 /topic_name example_interfaces/msg/String "{data: 'Hello'}"`
Example:
```
ros2 topic pub -r 2 /robot_news example_interfaces/msg/String "{data: 'Hello'}"
```

Rename a topic in runtime:
```
ros2 run my_py_pkg robot_news_station --ros-args -r __node:=my_station -r robot_news:=my_news
```
or 
```
ros2 run my_py_pkg robot_news_station --ros-args -r robot_news:=my_news
```

## Services

List the services:
`ros2 service list`

call service like a client:
`ros2 service call <service_name> <srv_type> '<request_args>'`
Example:
```
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

get service interface type:
`ros2 service type <service_name>`

The service can be called using the rqt plugin "service caller". It allows you to call the service trough an interactive GUI.

Remap the service in runtime:
`ros2 run my_cpp_package <node_name> --ros-args -r <service_name>:=<new_service_name>`

## Parameters

- Configuration value for a node, usefull for any kind of setting needed. 
- These are setted when starting the node or run-time.
- A parmeter is specific to a node.
- Has a name and a data type.

List each node with its parameters:
`ros2 param list`

Get the current value of an specific parameter:
`ros2 param get </node_name> <param_name>`

Start a node setting a parameter value:
`ros2 run <package_name> <node_name> --ros-args -p <param_name>:=<value>`

For multiple parameters you only have to add more -p: 
`ros2 run <package_name> <node_name> --ros-args -p <param1_name>:=<value> -p <param2_name>:=<value>`

## rqt and rqt_graph
This is GUI framework tool that is used to debug and understand better your graph, node, services, etc
Is a collection of pluggins that can be connected.

```bash
rqt
```

```bash
rqt_graph
```

## Turtlesim
It is a simulator package that allows you to interact with a graphic interface.




