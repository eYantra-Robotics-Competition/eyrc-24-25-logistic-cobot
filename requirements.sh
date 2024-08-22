#!/bin/bash

# Function to install packages
install_package() {
    sudo apt-get install -y "$1" > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo -e "\e[32mPackage: $1 installed successfully!\e[0m"
    else
        echo -e "\e[31mPackage: $1 failed to install:(\e[0m"
    fi
}

# Function to update files
update_files() {
    sudo cp "$2" "$3"
    if [ $? -eq 0 ]; then
        echo -e "\e[32mStatus: $1 updated successfully.\e[0m"
    else
        echo -e "\e[31mError: Failed to update $1.\e[0m"
        exit 1
    fi
}

# Install necessary packages
echo -e "\e[34mInstalling required packages...\e[0m"
install_package "python3-pip"
install_package "ros-humble-gazebo-ros"
install_package "ros-humble-gazebo-plugins"
install_package "ros-humble-xacro"
install_package "ros-humble-tf-transformations"

# Install transforms3d with a warning message
#echo -e "\e[34mInstalling transforms3d...\e[0m"
sudo pip3 install transforms3d > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo -e "\e[32mPackage: transforms3d installed successfully.\e[0m"
else
    echo -e "\e[33mWarning: Failed to install transforms3d. You may want to address this warning, but it won't halt the script.\e[0m"
fi

echo -e "\e[34mUpdating SkyX_Moon.fragment and SkyX_Moon.png for gazebo environment setup...\e[0m"
sudo cp eyantra_warehouse/config/SkyX_Moon.fragment /usr/share/gazebo-11/media/skyx/SkyX_Moon.fragment
sudo cp eyantra_warehouse/config/SkyX_Moon.png /usr/share/gazebo-11/media/skyx/SkyX_Moon.png

echo -e "\e[32m---------------------------------\nSetup complete!\n---------------------------------\n\e[0m"
