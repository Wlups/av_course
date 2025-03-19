**Requirements**
Before proceeding, ensure you have the following:

Ubuntu 22.04
ROS 2 Humble
Gazebo simulation environment
TurtleBot3 installed
Dependencies as outlined in the original tutorial


Step 1: Follow the tutorial provided by the teacher "Mapping.pdf"

Step 2: Clone the Repository
To get started, clone the repository to your local machine.

cd ~/ros2_ws/src
git clone https://github.com/Wlups/my_robot_controller.git
git clone https://github.com/Wlups/turtlebot3_simulations.git

Step 3: from ros2_ws/src/turtlebot3/simulations/turtlebot3/gazebo/launch/turtlebot3_world.launch.py

change following


    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'proov1.world'
    )

  And build the workspace and source it


  Step 4. launch with new modified launch file:
  
  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.p

  Step 5. in another terminal Launch SLAM
  
  ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

  Step 6. IN another terminal launch the mapping node

  ros2 run my_robot_controller mapping
  

