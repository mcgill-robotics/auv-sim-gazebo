#GAZEBO
sudo apt-get update
sudo apt-get install -y lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install -y ignition-fortress
rosdep install -r --from-paths src -i -y --rosdistro noetic
#ROS-IGN BRIDGE

echo "Select an installation method:"
echo "1: Install using apt (faster)" # apt everything except custom ros ign bridge
echo "2: Install from source" # build all ign packages from source (more reliable if first option causes errors)
export IGNITION_VERSION=fortress
auv_sim_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
while true; do
    read -p "Enter your choice: " choice

    if [ "$choice" == "1" ]; then
        sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
        sudo apt-get update
        sudo apt install -y ros-noetic-ros-ign
        cd "${auv_sim_dir}/../"
        git clone https://github.com/mcgill-robotics/auv-ros-ign-bridge
        break
    elif [ "$choice" == "2" ]; then
        cd "${auv_sim_dir}/../"
        git clone https://github.com/mcgill-robotics/auv-ign
        # If rosdep fails to install Ignition libraries and you have not installed them before, please follow Ignition installation instructions:
        #  https://gazebosim.org/docs/latest/install
        break
    else
        echo "Invalid choice. "
    fi
done

cd ..
source /opt/ros/noetic/setup.bash
rosdep install -r --from-paths src -i -y --rosdistro noetic

echo "Done."