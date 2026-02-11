To run the program on robotics lab machines:

1. Open new terminal and source the environment:
    - Put the following into the terminal: “source /opt/ros/jazzy/setup.bash”
2. Navigate to the root of the project workspace.
    - IMPORTANT:: Type in “code .” into the terminal to open VS Code with ros2
3. To build the project:
    - In the terminal: “colcon build --packages-select aiChairDocking_pkg”
    - IMPORTANT: Should be executed in the root of the project workspace. Should NOT be executed in the pkg folder
4. To run the project:
    - Open new terminal and navigate to root project directory
    - Then in the terminal type: “source ~/aiChairDocking/install/setup.bash”
    - Once this is done, type: ros2 run aiChairDocking_pkg publisher
    - repeat steps in step 4, then only the last step is different. Replace 'publisher' with 'subscriber' in the last command
