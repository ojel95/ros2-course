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

## Create executable to be installed (Python) 

1. Open the setup.py file of the package
2. Add to the 'console-scripts' the name of the executable and the entry point.
  `NAME_OF_EXEC = NAME_OF_PKG.NAME_OF_NODE_FILE:ENTRY_FUNCTION`
  Ex.
  `py_node = my_py_pkg.my_first_node:main`

This will create the output file in the path specified in the setup.cfg file. In this case it will be: 
ros2_ws/install/my_py_pkg/lib/my_py_pkg/py_node

3. Now you can build the pkg with: `colcon build --packages-select my_py_pkg`

## Run an installed node

ros2 run PACKAGE_NAME NODE_EXECUTABLE_NAME
Ex.
`ros2 run my_py_pkg py_node`

## Common ROS Node Functions

### crate_timer(period, callback)

This will create a timer where a callback will be called every given period.
