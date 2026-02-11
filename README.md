To run the program on robotics lab machines:

1. Open new terminal and source the environment:
    - Put the following into the terminal: “source /opt/ros/jazzy/setup.bash”
Navigate to the root of the project workspace.
IMPORTANT:: Type in “code .” into the terminal to open VS Code with ros2
To build the project:
In the terminal: “colcon build --packages-select aiChairDocking_pkg”
IMPORTANT: Should be executed in the root of the project workspace. Should NOT be executed in the pkg folder
To run the project:
Open new terminal and navigate to root project directory
Then in the terminal type: “source ~/aiChairDocking/install/setup.bash”
Ros2 run <name of package> <executable>
