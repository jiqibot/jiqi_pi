Articubot is where all the simulations for the robot are run. 

LAUNCH FILES commands:

roslaunch articubot hector_gazebo.launch
^^^^^
This runs hector slam, base local planner, move base and all the costmaps and local planners.

roslaunch articubot TEB_gazebo.launch
^^^^^
This runs hector slam, TEB local planner etc. TEB local planner is a better algorithm

roslaunch articubot gazebo_rsp.launch
^^^^^
This NO slam algorithm. A pre-generated map is served using map server package and AMCL is used for localisation. Move base is also integrated so you can set goals like the other launch files

The remainder launch files were used to help learn about ROS and linux during the project. I will leave them just in case you want to look at them. The main launch files are mentioned above
