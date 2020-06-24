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
  # open new terminal
  $ source /opt/ros/dashing/setup.bash
<<<<<<< HEAD
  $ cd../../.. (cd <ros1_ws>)
=======
  $ cd <ros1_ws>
>>>>>>> ebf7fcd8f11255db41af2958ec8c40aee973ce88
  $ . devel/setup.bash
  $ cd src/ros2/ros2_ws
  $ . install/setup.bash
  $ colcon build --symlink-install --packages-select ros1_bridge
```

# STARTUP

```bash
  $ . devel/setup.bash
  $ roslaunch strategy main_6th_all.launch
  $ roslaunch strategy core.launch
<<<<<<< HEAD
  $ cd src/ros2/ros2_ws
  $ . install/setup.bash
=======
  $ . src/ros2/ros2_ws/install/setup.bash
>>>>>>> ebf7fcd8f11255db41af2958ec8c40aee973ce88
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

