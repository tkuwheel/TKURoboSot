# BUILD

```bash
#-------compiling ros1---------
	$ source /opt/ros/melodic/setup.bash
	$ touch src/ros2/CATKIN_IGNORE
	$ catkin_make
#-------compiling ros2---------
	$ cd src/ros2/ros2_ws
	$ source /opt/ros/dashing/setup.bash
	$ colcon build --symlink-install --packages-select strategy
#---build custom ros1_bridge---
	$ source /opt/ros/dashing/setup.bash
	cd <ros1_ws>
	$ . devel/setup.bash
	# open new terminal
	cd src/ros2/ros2_ws
	$ . install/setup.bash
	$ colcon build --symlink-install --packages-select ros1_bridge
```

# STARTUP

```bash
	$ . devel/setup.bash
	$ roslaunch strategy main_6th_all.launch
	$ roslaunch strategy core.launch
	$ . src/ros2/ros2_ws/install/setup.bash
	$ ros2 run ros1_bridge parameter_bridge
```

# SIMULATOR  
  - install topic_tools  
    ```
    sudo apt-get install ros-topic-tools-srvs
    ```
  - start  
    ```bash
    # Gazebo Simulator
    $ roslaunch nubot_gazebo game_ready.launch

    # Strategy simulation mode
    $ roslaunch strategy core.launch sim:=true

    # GUI
    # using plugin of dynamic_reconfigure
    $ rqt
    ```

