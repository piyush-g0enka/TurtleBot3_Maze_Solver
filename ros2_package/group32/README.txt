
------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Steps to run the project
------------------------------------------------------------------------------------------------------------------------------------------------------------------

1. cd to the workspace, build the project "group32" with command 
	>_ colcon build --packages-select group32

2. Source the underlay

3. Launch the launch file from the turtlebot3_gazebo package with the following command
	>_ ros2 launch turtlebot3_gazebo maze.launch.py

4. Wait for Gazebo to launch properly

5. Open another terminal, launch our custom launch file from the "group32" package with the following command
	>_ ros2 launch group32 group32.launch.py
	
6. Now the robot should be moving, and relevant information will be printing out in the terminal.


==================================================================================================================================================================
===================================================================================================================================================================

------------------------------------------------------------------------------------------------------------------------------------------------------------------
	Troubleshooting
------------------------------------------------------------------------------------------------------------------------------------------------------------------

1. If the robot at some point is not moving as expected
	? At a certain point, the robot might be stuck in a rotating state and not able to continue the path.
	
	PLEASE RUN AGAIN
	
-> If this problem persists, in order to finish the project, please manually in gazebo do the following:
	1. Launch the Gazebo with >_ ros2 launch turtlebot3_gazebo maze.launch.py
	2. Within gazebo, move the robot using the object move, place it at the relative location where it got stuck.
	3. Open another terminal, launch our custom launch file with >_ ros2 launch group32 group32.launch.py

===================================================================================================================================================================



