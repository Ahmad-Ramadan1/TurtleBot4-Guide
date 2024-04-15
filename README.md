# Implementation of TurtleBot with ROS2: Development and Integration

# Background:

ROS 2 stands for "Robot Operating System 2". It's an open-source framework for writing robot software. It's designed to be a flexible platform for developing robotics applications. ROS 2 is a successor to ROS, aiming to address limitations and improve upon its predecessor's functionalities. It offers improved real-time capabilities, better support for various hardware platforms, and enhanced security features, among other advancements. 
ROS 2 relies on the notion of combining workspaces using the shell environment. “Workspace” is a ROS term for the location on your system where you’re developing with ROS 2. The core ROS 2 workspace is called the underlay. Subsequent local workspaces are called overlays. When developing with ROS 2, you will typically have several workspaces active concurrently. 
Combining workspaces makes developing against different versions of ROS 2, or against different sets of packages, easier. It also allows the installation of several ROS 2 distributions (distros) on the same computer and switching between them. 
This is accomplished by sourcing setup files every time you open a new shell, or by adding the source command to your shell startup script once. Without sourcing the setup files, you won’t be able to access ROS 2 commands, or find or use ROS 2 packages. In other words, you won’t be able to use ROS 2.

# TurtleBot4
The TurtleBot 4 is a ROS 2-based mobile robot intended for education and research. The TurtleBot 4 is capable of mapping the robot's surroundings, navigating autonomously, running AI models on its camera, and more. 
It uses a Create® 3 as the base platform, and builds on it with the TurtleBot 4 shell and User Interface (UI) board. Inside the shell sits a Raspberry Pi 4B which runs the TurtleBot 4 software. 
The TurtleBot 4 Lite is a barebones version of the TurtleBot 4. It has just the necessary components for navigation, mapping, and AI applications. The TurtleBot 4 has the same Raspberry Pi 4B, which sits in the cargo bay of the Create® 3, as well as the same RPLIDAR A1M8. The camera on the TurtleBot 4 Lite is the OAK-D-Lite. Additional sensors and payloads can be attached to the Create® 3 faceplate, or placed inside the cargo bay.

# Sensors

1) RPLIDAR A1M8
The RPLIDAR A1M8 is a 360 degree Laser Range Scanner with a 12m range. It is used to generate a 2D scan of the robot's surroundings.
        
2)  OAK-D-Pro
The OAK-D-Lite camera from Luxonis uses a 4K IMX214 colour sensor along with a pair of OV7251 stereo sensors to produce high quality colour and depth images. 
The on-board Myriad X VPU gives the camera the power to run computer vision applications, object tracking, and run AI models.
        
3) OAK-D-Lite
The OAK-D-Pro offers all of the same features the OAK-D-Lite has, but uses higher resolution OV9282 stereo sensors and adds an IR laser dot projector and an IR illumination LED. This allows the camera to create higher quality depth images, and perform better in low-light environments.

In our project, we are currently focusing on the development and integration of the TurtleBot 4. Below, you can find a table highlighting the key differences between the TurtleBot 4 and the TurtleBot 4 Lite. This comparison will help provide insight into the unique features and specifications of each model, aiding in decision-making and understanding their respective capabilities within our project.

# Installing ROS2

## 1) Implementation of the UTF-8 Format
```bash
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```
    
We observe the installation with the "locale" command

## 2) Source configuration
```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee/etc/apt/sources.list.d/ros2.list /dev/null
```

## 3) Installing ROS2 packages
    
### I) Apt update
```bash
sudo apt update
sudo apt upgrade
```
### II) Installing ROS2 packages 
To install ROS2, there are 2 choices:
a) the full version including graphics libraries, example codes, etc.;
b) the “lite” version with only what is needed to run ROS2
           
The second installation is often used for systems where resources are “limited” like a RaspberryPI for example.
Installing ROS2 packages:
```bash
sudo apt install ros-humble-desktop
```            
Installation of ROS2 Limited:
```bash
sudo apt install ros-humble-ros-base
```            
If you go to the official ROS2 Humble installation site, they present a "Development Tool" when installing ROS2. You have the option of installing it for your own reasons. In this document, we don't use it.

### III) ROS2 environment configuration
To use ROS2, it is necessary to "source" its installation folder in order to use ROS2 commands in terminals. We modify the "bashrc" script as follows.
```bash
gedit ~/.bashrc
```
A text editor window should open. At the end of this text file the following lines and save. The second line enables systems using ROS2 to communicate via WIFI on a network described by the ROS domain ID (default '0').
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
```
To update the .bashrc file in your terminal, run the following command.
```bash
source ~/.bashrc
```
Note: You can also restart your terminal

### IV) ROS2 installation test
Test nodes are available to verify ROS2 installation. Run two different terminals and run the following commands.
    
#### a) Terminal 1:
```bash
ros2 run demo_nodes_cpp talker
```    
#### b) Terminal 2:
```bash
ros2 run demo_nodes_cpp listener
```
You need to get similar results (the terminals talk to each other).

### V) Installing the colcon compiler
The colcon compiler can be used to build a ROS2 application.
Here are the commands to write in a terminal.
```bash
sudo apt update
sudo apt install python3-colcon-common-extensions
```
To make the compiler easier to use, we mouse over the compiler path in the .bashrc file as :
```bash
gedit ~/.bashrc
```
Then, in this file, below the ROS2 sourcing in the "ROS2 installation" section, we write the following line and save it. "ROS2 installation" section, we write the following line and save.
 ```bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
```
To update the .bashrc file in your terminal, run the following command.
```bash
source ~/.bashrc
```
Note: You can also restart your terminal


# Turtlebot4 installation and configuration:

## 1) Preparing the WIFI network
Firstly, it is important to configure a network to support at least two wifi bands (2.4GHz and 5GHz). The minimum hardware needed to monitor a turtlebot4 is:
A WIFI terminal (or cell phone access point) connected to the Internet
A computer running Linux Ubuntu 22.04 with ROS2 for supervision purposes       
Turtlebot4
In the case of a network not connected to the Internet, you'll need another computer, preferably running Linux Ubuntu 22.04. The purpose of this computer is to synchronize all equipment connected to the network by broadcasting the date and time using the NTP time protocol.

## 2) ROS2 package installation for turtlebot4
On the supervision computer, open a terminal and enter the following commands:
```bash
sudo apt update && sudo apt install ros-humble-turtlebot4-desktop
sudo apt install ros-humble-turtlebot4-description 
ros-humble-turtlebot4-msgs 
ros-humble-turtlebot4-navigation 
ros-humble-turtlebot4-node
```
## 3) Turtlebot4 configuration
To configure turtlebot4 correctly, it's important to keep the system up to date with the latest patches. To do this, we first update the turtlebot's components, and then configure their parameters.

### a) RaspberryPi firmware update 
To update the RaspberryPI, you need to extract the micro SD card from the RaspberryPI card. 
We now use a micro SD to SD adapter and insert the card into our into the SD drive of our computer running Linux Ubuntu 22.04. 
We open the 'disques' or 'disks' utility and select the SD card. We format overwriting existing data with zeros and without partitioning. 
We download the latest update from the following site, taking into account the version of ROS2 (here Humble): http://download.ros.org/downloads/turtlebot4/
We extract the .img file from the downloaded .zip file and enter the following command in a terminal.
```bash
wget https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/humble/scripts/sd_flash.sh
bash sd_flash.sh /chemin/de/l’image.img
```
To find the name of the SD card, go to the 'disque' or 'disks' utility select the SD card and find the name next to 'Device' (in our case, we have /dev/mmcblk0'. 
When the terminal asks for the SD card name, we enter 'mmcblk0' and continue. 
We put the SD card back into turtlebot4 and reassemble the robot.
            
### b) CREATE3 card update 
Switch on the turtlebot4 by positioning it on its charging base connected to the mains and wait for the robot to play a sound. \\
On the supervision computer, download the latest version of Create3 (here Humble H2.6). Then connect to the turtlebot4's wifi network (SSID: 'Turtlebot4' | Mdp: 'Turtlebot4'). \\
Go to a web browser and enter in the address bar the ip '10.42.0.1:8080'. In the 'Update' tab, follow the update instructions.

### c) RaspberryPi configuration 
Switch on turtlebot4 by positioning it on its charging base connected to the mains and wait for the robot to broadcast a sound. \\
On the supervision computer, connect to the turtlebot4's wifi network (SSID: 'Turtlebot4' | Mdp: 'Turtlebot4'). Go to the remote access session of the turtlebot4 by typing the following command in a terminal:
```bash
ssh ubuntu@10.42.0.1
```
The session password is 'turtlebot4'. 
Once connected to turtlebot4, type the following command to configure the robot's parameters.
```bash
turtlebot4-setup
```
A graphical interface opens.
Go to WIFI-SETUP and connect the robot as a client to your wifi network (it's more optimal to connect the turtlebot4 to 5GHz).
Save and apply changes (you'll be disconnected from the turtlebot's internal wifi). 
Connect to your wifi network and go to the remote access session of the turtlebot4 by typing the following command in a terminal:
ssh ubuntu@’ip ecrite sur le turtlebot4’

If you have a lite version of turtlebot4, type the command 'ros2 topic echo /ip' on your supervision PC to find out the ip of the robot connected to your network. 
If you've mistakenly written the SSID and wifi password on the turtlebot4, please visit this site: 
https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html . 
If you wish to modify the \texttt{ROS\_DOMAIN\_ID} variable, go to the \texttt{ROS-SETUP} menu, then \texttt{BASH-SETUP}.
Save and apply the changes.

### d) Create3 card configuration 
Go to the turtlebot4 online space with a browser and the following ip: robot ip':8080 and connect the create3 card to your wifi network using the 'connect' tab.


Click 'connect' and the robot restarts. 

In the 'Application' tab, then 'Configuration', you can modify the ROS2 parameters (ensure that these parameters are identical to those on the RaspberryPi board). 


Restart the robot (to switch it off, remove it from the base and press and hold the stop button).

## 4) NTP protocol configuration (clock synchronization)
For the proper operation of the network and the systems installed on it, it's important to synchronize everything to the same time and date. In fact, some community-programmed ROS2 nodes use time-stamped data for their operation their operation. 
So we configure the routes and servers to be reached to update the time and date of every date of each device on the wifi network. 

We recommend setting the NTP relay to a fixed IPv4 address. In our case, we manually set these parameters on the ENDORSE PC connected to the network as follows: 
IP:192.168.1.32 | Subnet mask: 255.255.255.0 | Default gateway:192.168.1.1 

### a) NTP relay configuration under Linux Ubuntu 22.04
#### i) Time zone change 
```bash
sudo timedatectl set-timezone UTC
```
#### ii) Installation of 'ntp' configuration software 
```bash
sudo apt update && sudo apt install ntp
```
#### iii) Configuring the ntp.conf file 
```bash
sudo nano /etc/ntp.conf
```
We save the file and restart the service.
```bash
sudo systemctl restart ntp
```
### b) Customer configuration 
#### i) Time zone change
```bash
sudo timedatectl set-timezone UTC
```
#### ii) Disabling the current ntp manager
```bash
sudo timedatectl set-ntp off
```
#### iii) Installation of 'ntp' and 'ntpdate' configuration and synchronization software
```bash
sudo apt update && sudo apt install ntp && sudo apt install ntpdate
```
#### iv) Configuring the ntp.conf file
```bash
sudo nano /etc/ntp.conf
```
For the Create3 client, we go to the Web interface as \texttt{{[}IP\_of\_robot{]}:8080}. \\
Here we use the following IP '192.168.1.7:8080'.

We enter the 'Edit ntp.conf' menu and enter the following lines (commenting on the default servers).

### c) Restart the ntp service
#### i) On linux:
```bash
sudo timedatectl set-timezone UTC
```
#### ii) On Create3: 

### d) Possible error 
If the RaspberryPI does not synchronize automatically when the robot is restarted, run the following command to do so manually.
```bash
sudo timedatectl set-timezone UTC && sudo ntpdate [ip_du_relai]
```
and enter this command in ~/.bashrc.

