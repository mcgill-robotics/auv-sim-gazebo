#GAZEBO
sudo apt-get update
sudo apt-get install -y lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y ignition-fortress
rosdep install -r --from-paths src -i -y --rosdistro noetic
#ROS-IGN BRIDGE
export IGNITION_VERSION=fortress
auv_sim_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
cd "${auv_sim_dir}/../../../"
git clone https://github.com/mcgill-robotics/auv-ign
cd auv-ign
rosdep install -r --from-paths src -i -y --rosdistro noetic
source /opt/ros/noetic/setup.bash
# If rosdep fails to install Ignition libraries and you have not installed them before, please follow Ignition installation instructions:
#  https://gazebosim.org/docs/latest/install
catkin_make install