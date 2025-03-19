**Requirements**
Before proceeding, ensure you have the following:

Ubuntu 22.04
ROS 2 Humble
Gazebo simulation environment
TurtleBot3 installed
Dependencies as outlined in the original tutorial




Step 1: Clone the Repository
To get started, clone the repository to your local machine.

cd ~/ros2_ws/src
git clone https://github.com/Wlups/my_robot_controller.git
git clone https://github.com/Wlups/turtlebot3_simulations.git

Step 2: Install Required Packages
Once the repositories are cloned, ensure you have all the necessary dependencies installed. In your terminal, run the following commands:

sudo apt install ros-humble-gazebo-*
sudo apt install ros-humble-turtlebot3-msgs
sudo apt install ros-humble-turtlebot3
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2
sudo apt install ros-humble-nav2-bringup

Add TurtleBot3 Model and ROS Domain to .bashrc

echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=30' >> ~/.bashrc
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
source ~/.bashrc

cd ~/ros2_ws
source ~/.bashrc

cd ~/ros2_ws
colcon build --symlink-install
source ~/.bashrc
