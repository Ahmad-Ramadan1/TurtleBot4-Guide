# TurtleBot4-Guide
```latex ... ```
Implementation of TurtleBot with ROS2: Development and Integration

Background:

ROS 2 stands for "Robot Operating System 2". It's an open-source framework for writing robot software. It's designed to be a flexible platform for developing robotics applications. ROS 2 is a successor to ROS, aiming to address limitations and improve upon its predecessor's functionalities. It offers improved real-time capabilities, better support for various hardware platforms, and enhanced security features, among other advancements. \\
ROS 2 relies on the notion of combining workspaces using the shell environment. “Workspace” is a ROS term for the location on your system where you’re developing with ROS 2. The core ROS 2 workspace is called the underlay. Subsequent local workspaces are called overlays. When developing with ROS 2, you will typically have several workspaces active concurrently. \\
Combining workspaces makes developing against different versions of ROS 2, or against different sets of packages, easier. It also allows the installation of several ROS 2 distributions (distros) on the same computer and switching between them. \\
This is accomplished by sourcing setup files every time you open a new shell, or by adding the source command to your shell startup script once. Without sourcing the setup files, you won’t be able to access ROS 2 commands, or find or use ROS 2 packages. In other words, you won’t be able to use ROS 2.

\subsubsection{TurtleBot4}
The TurtleBot 4 is a ROS 2-based mobile robot intended for education and research. The TurtleBot 4 is capable of mapping the robot's surroundings, navigating autonomously, running AI models on its camera, and more. \\
It uses a Create® 3 as the base platform, and builds on it with the TurtleBot 4 shell and User Interface (UI) board. Inside the shell sits a Raspberry Pi 4B which runs the TurtleBot 4 software. \\
The TurtleBot 4 Lite is a barebones version of the TurtleBot 4. It has just the necessary components for navigation, mapping, and AI applications. The TurtleBot 4 has the same Raspberry Pi 4B, which sits in the cargo bay of the Create® 3, as well as the same RPLIDAR A1M8. The camera on the TurtleBot 4 Lite is the OAK-D-Lite. Additional sensors and payloads can be attached to the Create® 3 faceplate, or placed inside the cargo bay.

\begin{itemize}

    \item \textbf{Sensors:} \\
    
    \item[] \hspace{1em} \textbf{1)} \textbf{RPLIDAR A1M8} 
    \begin{itemize}
        \begin{minipage}{0.65\textwidth}
            \item The RPLIDAR A1M8 is a 360 degree Laser Range Scanner with a 12m range. It is used to generate a 2D scan of the robot's surroundings.
        \end{minipage}
        \begin{minipage}{0.3\textwidth}
            \includegraphics[width=\linewidth]{Images/rplidar_a1m8.png}
        \end{minipage}
    \end{itemize}


    \item[] \hspace{1em} \textbf{2)} \textbf{OAK-D-Pro} 
    \begin{itemize}
        \begin{minipage}{0.65\textwidth}
            \item The OAK-D-Lite camera from Luxonis uses a 4K IMX214 colour sensor along with a pair of OV7251 stereo sensors to produce high quality colour and depth images. The on-board Myriad X VPU gives the camera the power to run computer vision applications, object tracking, and run AI models.
        \end{minipage}
        \begin{minipage}{0.3\textwidth}
            \includegraphics[width=\linewidth]{Images/oak-d-pro.png}
        \end{minipage}
    \end{itemize}

    \item[] \hspace{1em} \textbf{3)} \textbf{OAK-D-Lite} 
    \begin{itemize}
        \begin{minipage}{0.65\textwidth}
            \item The OAK-D-Pro offers all of the same features the OAK-D-Lite has, but uses higher resolution OV9282 stereo sensors and adds an IR laser dot projector and an IR illumination LED. This allows the camera to create higher quality depth images, and perform better in low-light environments.
        \end{minipage}
        \begin{minipage}{0.3\textwidth}
            \includegraphics[width=\linewidth]{Images/oak-d-lite.png}
        \end{minipage}
    \end{itemize}

\end{itemize}

In our project, we are currently focusing on the development and integration of the TurtleBot 4. Below, you can find a table highlighting the key differences between the TurtleBot 4 and the TurtleBot 4 Lite. This comparison will help provide insight into the unique features and specifications of each model, aiding in decision-making and understanding their respective capabilities within our project.

\begin{table}
\centering
\caption{Comparison of TurtleBot 4 Lite and TurtleBot 4}
\label{tab:comparison}
\begin{tabular}{@{}lll@{}}
\toprule
  & \begin{tabular}[c]{@{}l@{}} \textbf{TurtleBot 4 Lite}\end{tabular} & \begin{tabular}[c]{@{}l@{}} \textbf{TurtleBot 4}\end{tabular} \\
\hline 
\multicolumn{3}{c}{\textbf{Weight and Size}} \\ \midrule
Dimensions External (LxWxH) & \begin{tabular}[c]{@{}l@{}}7.5 x 13.3 x 13.4 mm \\ (192 x 339 x 341 in)\end{tabular} & \begin{tabular}[c]{@{}l@{}}13.8 x 13.3 x 13.4 mm \\ (351 x 339 x 341 in)\end{tabular} \\
Weight                      & \begin{tabular}[c]{@{}l@{}}7.2 lbs \\ (3.3 kg)\end{tabular}                              & \begin{tabular}[c]{@{}l@{}}8.6 lbs \\ (3.9 kg)\end{tabular}                              \\
Wheels Diameter             & \begin{tabular}[c]{@{}l@{}}0.55 in \\ (14 mm)\end{tabular}                               & \begin{tabular}[c]{@{}l@{}}0.55 in \\ (14 mm)\end{tabular}                               \\
Ground Clearance            & \begin{tabular}[c]{@{}l@{}}0.17 in \\ (4.5 mm)\end{tabular}                              & \begin{tabular}[c]{@{}l@{}}0.17 in \\ (4.5 mm)\end{tabular}                              \\ \midrule
\multicolumn{3}{c}{\textbf{Performance and Speed}}                                                \\ \midrule
Payload Max (Default)       & 9 kg                                           & 15 kg (Custom Configuration)                  \\
Max Speed                   & \begin{tabular}[c]{@{}l@{}}0.46 m/s \\ (Safe Mode)\end{tabular}                          & \begin{tabular}[c]{@{}l@{}}0.31 m/s \\ (Safe Mode)\end{tabular}                          \\
Max Rotational Speed        & 1.90 s/rad                                     & 1.90 s/rad                                    \\ \midrule
\multicolumn{3}{c}{\textbf{System Power and Battery}}                                              \\ \midrule
Chemistry                   & Lithium Ion                                    & Lithium Ion                                   \\
Nominal Voltage             & 14.4 V                                         & 14.4 V                                        \\
Battery Capacity            & 26 Wh                                          & 26 Wh                                         \\
Charge Time                 & 2.5 hrs                                        & 2.5 hrs                                       \\
Operating Time              & 4.0-2.5 hrs (Dependent on Load)               & 4.0-2.5 hrs (Dependent on Load)              \\ \midrule
\multicolumn{3}{c}{\textbf{Sensors}}                                                                \\ \midrule
LIDAR                       & 1A-RPLIDAR                                     & 1A-RPLIDAR                                    \\
Camera                      & PRO-D-OAK                                      & PRO-D-OAK                                     \\
Other Sensors               & LITE-D-OAK                                     & LITE-D-OAK                                    \\ \midrule
\multicolumn{3}{c}{\textbf{Actuators and Computers}}                                                \\ \midrule
Actuators                   & Motors Drive x2                                & Motors Drive x2                               \\
Computers                   & Pi Raspberry (4 GB)                            & Pi Raspberry (4 GB)                           \\ \midrule
\multicolumn{3}{c}{\textbf{Software}}                                                                \\ \midrule
ROS Version                 & 2.0.04                                         & 2.0.04                                        \\
Operating System            & Ubuntu                                         & Ubuntu                                        \\ \bottomrule
\end{tabular}
\end{table}

\newpage

\begin{itemize}

    \item \textbf{Installing ROS2:} \\
    
    \item[] \hspace{1em} \textbf{1)} \textbf{Implementation of the UTF-8 Format} 
    \begin{verbatim}
    > sudo apt update && sudo apt install locales
    > sudo locale-gen en_US en_US.UTF-8
    > sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    > export LANG=en_US.UTF-8
    \end{verbatim}
    \hspace{0.9cm} We observe the installation with the "locale" command
    \begin{figure}[h]
        \centering
        \includegraphics[width=0.5\textwidth]{Images/Locale Command.jpg}
         \caption{locale Command}
        \label{fig: locale Command}
    \end{figure}

    \item[] \hspace{1em} \textbf{2)} \textbf{Source configuration}
    \begin{verbatim}
    > sudo apt install software-properties-common
    > sudo add-apt-repository universe
    > sudo apt update && sudo apt install curl -y
    > sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key 
    -o /usr/share/keyrings/ros-archive-keyring.gpg
    > echo "deb [arch=$(dpkg --print-architecture) 
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg]
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release 
    && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    \end{verbatim}
        
    \newpage

    \item[] \hspace{1em} \textbf{3)} \textbf{Installing ROS2 packages}
    \begin{itemize}
    \item Apt update
    \end{itemize}
    \begin{verbatim}
    > sudo apt update
    > sudo apt upgrade
    \end{verbatim}

    \begin{itemize}
    \item Installing ROS2 packages 
        \begin{itemize}
        \item To install ROS2, there are 2 choices:
            \begin{itemize}
            \item the full version including graphics libraries, example codes, etc.;
    
            \item the “lite” version with only what is needed to run ROS2
            \end{itemize}
        The second installation is often used for systems where resources are “limited” like a RaspberryPI for example.
        \item Installing ROS2 packages:
\begin{verbatim}
> sudo apt install ros-humble-desktop
\end{verbatim}
        \item Installation of ROS2 Limited:
\begin{verbatim}
> sudo apt install ros-humble-ros-base
\end{verbatim}
        If you go to the official ROS2 Humble installation site, they present a "Development Tool" when installing ROS2. You have the option of installing it for your own reasons. In this document, we don't use it.
        \end{itemize}
    \end{itemize}

    \item[] \hspace{1em} \textbf{4)} \textbf{ROS2 environment configuration}
     \begin{itemize}
     \item To use ROS2, it is necessary to "source" its installation folder in order to use ROS2 commands in terminals. We modify the "bashrc" script as follows.
\begin{verbatim}
> gedit ~/.bashrc
\end{verbatim}
     A text editor window should open. At the end of this text file the following lines and save. The second line enables systems using ROS2 to communicate via WIFI on a network described by the ROS domain ID (default '0').
\begin{verbatim}
> source /opt/ros/humble/setup.bash
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> export ROS_DOMAIN_ID=0
\end{verbatim}
     To update the .bashrc file in your terminal, run the following command.
\begin{verbatim}
> source ~/.bashrc
\end{verbatim}
     \textbf{Note}: You can also restart your terminal
     \end{itemize}

\newpage

    \item[] \hspace{1em} \textbf{5)} \textbf{ROS2 installation test}
     \begin{itemize}
     \item Test nodes are available to verify ROS2 installation. Run two different terminals and run the following commands.

        \begin{itemize}
        \item Terminal 1:
\begin{verbatim}
> ros2 run demo_nodes_cpp talker
\end{verbatim}

        \item Terminal 2:
\begin{verbatim}
> ros2 run demo_nodes_cpp listener
\end{verbatim}
        \end{itemize}
     You need to get similar results (the terminals talk to each other).
    \end{itemize}

    \begin{figure}[h]
        \centering
        \includegraphics[width=0.9\textwidth]{Images/Talker-Listener.jpg}
         \caption{Talker-Listener}
        \label{fig: Talker-Listener}
    \end{figure}

    \item[] \hspace{1em} \textbf{6)} \textbf{Installing the colcon compiler}
     \begin{itemize}
     \item The colcon compiler can be used to build a ROS2 application.\\
     Here are the commands to write in a terminal.
\begin{verbatim}
> sudo apt update
> sudo apt install python3-colcon-common-extensions
\end{verbatim}
     To make the compiler easier to use, we mouse over the compiler path in the .bashrc file as :
\begin{verbatim}
> gedit ~/.bashrc
\end{verbatim}
     Then, in this file, below the ROS2 sourcing in the "ROS2 installation" section, we write the following line and save it. "ROS2 installation" section, we write the following line and save.
\begin{verbatim}
> source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
\end{verbatim}
     To update the .bashrc file in your terminal, run the following command.
\begin{verbatim}
> source ~/.bashrc
\end{verbatim}
     \textbf{Note}: You can also restart your terminal
     \end{itemize}

\end{itemize}

\newpage

\begin{itemize}
    \item \textbf{Turtlebot4 installation and configuration:} \\

    \item[] \hspace{1em} \textbf{1)} \textbf{Preparing the WIFI network} 
    \begin{itemize}
    \item Firstly, it is important to configure a network to support at least two wifi bands (2.4GHz and 5GHz). The minimum hardware needed to monitor a turtlebot4 is:
        \begin{itemize}
        \item A WIFI terminal (or cell phone access point) connected to the Internet
        \item A computer running Linux Ubuntu 22.04 with ROS2 for supervision purposes       
        \item Turtlebot4
        \end{itemize}
    In the case of a network not connected to the Internet, you'll need another computer, preferably running Linux Ubuntu 22.04. The purpose of this computer is to synchronize all equipment connected to the network by broadcasting the date and time using the NTP time protocol.
    \end{itemize}

    \item[] \hspace{1em} \textbf{2)} \textbf{ROS2 package installation for turtlebot4}
    \begin{itemize}
    \item On the supervision computer, open a terminal and enter the following commands:
    \end{itemize}
    \begin{verbatim}
    > sudo apt update && sudo apt install ros-humble-turtlebot4-desktop
    > sudo apt install ros-humble-turtlebot4-description 
    > ros-humble-turtlebot4-msgs 
    > ros-humble-turtlebot4-navigation 
    > ros-humble-turtlebot4-node
    \end{verbatim}

    \item[] \hspace{1em} \textbf{3)} \textbf{Turtlebot4 configuration}
    \begin{itemize}
    \item To configure turtlebot4 correctly, it's important to keep the system up to date with the latest patches. To do this, we first update the turtlebot's components, and then configure their parameters.
        \begin{itemize}
            \item RaspberryPi firmware update \\
            To update the RaspberryPI, you need to extract the micro SD card from the RaspberryPI card. \\
            We now use a micro SD to SD adapter and insert the card into our into the SD drive of our computer running Linux Ubuntu 22.04. \\
            We open the 'disques' or 'disks' utility and select the SD card. We format overwriting existing data with zeros and without partitioning. \\
            We download the latest update from the following site, taking into account the version of ROS2 (here Humble): http://download.ros.org/downloads/turtlebot4/ \\
            We extract the .img file from the downloaded .zip file and enter the following command in a terminal.
\begin{verbatim}
> wget https://raw.githubusercontent.com/turtlebot/turtlebot4_setup/humble/
scripts/sd_flash.sh
> bash sd_flash.sh /chemin/de/l’image.img
\end{verbatim}
            To find the name of the SD card, go to the 'disque' or 'disks' utility select the SD card and find the name next to 'Device' (in our case, we have /dev/mmcblk0'. \\
            When the terminal asks for the SD card name, we enter 'mmcblk0' and continue. \\
            We put the SD card back into turtlebot4 and reassemble the robot.
            
            \item CREATE3 card update \\
            Switch on the turtlebot4 by positioning it on its charging base connected to the mains and wait for the robot to play a sound. \\
            On the supervision computer, download the latest version of Create3 (here Humble H2.6). Then connect to the turtlebot4's wifi network (SSID: 'Turtlebot4' | Mdp: 'Turtlebot4'). \\
            Go to a web browser and enter in the address bar the ip '10.42.0.1:8080'. In the 'Update' tab, follow the update instructions.

            \begin{figure}[h]
            \centering
            \includegraphics[width=0.65\textwidth]{Images/Update Robot.jpg}
            \caption{Update Robot}
            \label{fig: Update Robot}
            \end{figure} 
            
            \item RaspberryPi configuration \\
            Switch on turtlebot4 by positioning it on its charging base connected to the mains and wait for the robot to broadcast a sound. \\
            On the supervision computer, connect to the turtlebot4's wifi network (SSID: 'Turtlebot4' | Mdp: 'Turtlebot4'). Go to the remote access session of the turtlebot4 by typing the following command in a terminal:
\begin{verbatim}
> ssh ubuntu@10.42.0.1
\end{verbatim}
            The session password is 'turtlebot4'. \\
            Once connected to turtlebot4, type the following command to configure the robot's parameters.
\begin{verbatim}
> turtlebot4-setup
\end{verbatim}
            A graphical interface opens. \\
            Go to WIFI-SETUP and connect the robot as a client to your wifi network (it's more optimal to connect the turtlebot4 to 5GHz).
            Save and apply changes (you'll be disconnected from the turtlebot's internal wifi). \\
            Connect to your wifi network and go to the remote access session of the turtlebot4 by typing the following command in a terminal:
\begin{verbatim}
> ssh ubuntu@’ip ecrite sur le turtlebot4’
\end{verbatim}
            If you have a lite version of turtlebot4, type the command '> ros2 topic echo /ip' on your supervision PC to find out the ip of the robot connected to your network. \\
            If you've mistakenly written the SSID and wifi password on the turtlebot4, please visit this site: \\
            https://turtlebot.github.io/turtlebot4-user-manual/setup/basic.html . \\
            If you wish to modify the \texttt{ROS\_DOMAIN\_ID} variable, go to the \texttt{ROS-SETUP} menu, then \texttt{BASH-SETUP}. \\ Save and apply the changes.

             \item Create3 card configuration \\
             Go to the turtlebot4 online space with a browser and the following ip: robot ip':8080 and connect the create3 card to your wifi network using the 'connect' tab.
             \begin{figure}[h]
             \centering
             \includegraphics[width=0.65\textwidth]{Images/Connect Robot to WIFI.jpg}
             \caption{Connect Robot to WIFI}
             \label{fig: Connect Robot to WIFI}
             \end{figure} 
             
             Click 'connect' and the robot restarts. \\
              
             \newpage
              
             In the 'Application' tab, then 'Configuration', you can modify the ROS2 parameters (ensure that these parameters are identical to those on the RaspberryPi board). \\

             \begin{figure}[h]
             \centering
             \includegraphics[width=0.65\textwidth]{Images/App Config.jpg}
             \caption{Main Configuration}
             \label{fig: Main Configuration}
             \end{figure} 

             Restart the robot (to switch it off, remove it from the base and press and hold the stop button).
             
        \end{itemize}
    \end{itemize}
           
\newpage

    \item[] \hspace{1em} \textbf{4)} \textbf{NTP protocol configuration (clock synchronization)}
    \begin{itemize}
    \item For the proper operation of the network and the systems installed on it, it's important to synchronize everything to the same time and date. In fact, some community-programmed ROS2 nodes use time-stamped data for their operation their operation. \\
    So we configure the routes and servers to be reached to update the time and date of every date of each device on the wifi network. \\
   
    \item We recommend setting the NTP relay to a fixed IPv4 address. In our case, we manually set these parameters on the ENDORSE PC connected to the network as follows: \\
    IP:192.168.1.32 | Subnet mask: 255.255.255.0 | Default gateway:
    192.168.1.1 \\
        \begin{itemize}
            \item NTP relay configuration under Linux Ubuntu 22.04\\
                \begin{itemize}
                    \item Time zone change 
\begin{verbatim}
>sudo timedatectl set-timezone UTC
\end{verbatim}
    
                    \item Installation of 'ntp' configuration software 
\begin{verbatim}
>sudo apt update && sudo apt install ntp
\end{verbatim}

                    \item Configuring the ntp.conf file 
\begin{verbatim}
>sudo nano /etc/ntp.conf
\end{verbatim}
                    We save the file and restart the service.
\begin{verbatim}
>sudo systemctl restart ntp
\end{verbatim}
                \end{itemize} 

            \item Customer configuration \\
                \begin{itemize}
                    \item Time zone change 
\begin{verbatim}
>sudo timedatectl set-timezone UTC
\end{verbatim}

                    \item Disabling the current ntp manager 
\begin{verbatim}
>sudo timedatectl set-ntp off
\end{verbatim}

                    \item Installation of 'ntp' and 'ntpdate' configuration and synchronization software 
\begin{verbatim}
>sudo apt update && sudo apt install ntp && sudo apt install ntpdate
\end{verbatim}

                    \item Configuring the ntp.conf file 
\begin{verbatim}
>sudo nano /etc/ntp.conf
\end{verbatim}
                \end{itemize}
                                   
        \end{itemize}
     
    \end{itemize}

\newpage

            \begin{itemize} 
                \item For the Create3 client, we go to the Web interface as \texttt{{[}IP\_of\_robot{]}:8080}. \\
                Here we use the following IP '192.168.1.7:8080'.
    
                \begin{figure}[h]
                \centering
                \includegraphics[width=0.65\textwidth]{Images/Create3 Web page.jpg}
                \caption{Create3 Web page}
                \label{fig: Create3 Web page}
                \end{figure}
    
                 \item We enter the 'Edit ntp.conf' menu and enter the following lines (commenting on the default servers).
    
                \begin{figure}[h]
                \centering
                \includegraphics[width=0.65\textwidth]{Images/NTP Config.jpg}
                \caption{NTP Configuration}
                \label{fig: NTP Configuration}
                \end{figure}
    
                \begin{itemize}
                    \item Restart the ntp service
                        \begin{itemize}
                            \item On linux: 
\begin{verbatim}
>sudo timedatectl set-timezone UTC
\end{verbatim} 
                            \item On Create3: 
                            
                            \begin{figure}[h]
                            \centering
                            \includegraphics[width=0.65\textwidth]{Images/Restart NTP Create3.jpg}
                            \caption{Restart NTP Create3}
                            \label{fig: Restart NTP Create3}
                            \end{figure}
                        \end{itemize}

\newpage

                    \item Possible error \\
                    If the RaspberryPI does not synchronize automatically when the robot is restarted, run the following command to do so manually.
\begin{verbatim}
>sudo timedatectl set-timezone UTC && sudo ntpdate [ip_du_relai]
\end{verbatim}
                    and enter this command in ~/.bashrc.
                 
                \end{itemize}
            \end{itemize}
\end{itemize}
