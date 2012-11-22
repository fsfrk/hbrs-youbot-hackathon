## Install Ubuntu
The repository and its related components have been tested under Ubuntu 10.04 LTS. If you do not have a Ubuntu distribution on you computer you can download it here

     http://www.ubuntu.com/download

## Git - Version Control
### Install Git Software
Install the Git core components and some additional GUI's for the version control:

     sudo apt-get install git-core gitg gitk

### Set Up Git
Now it's time to configure your settings. To do this you need to open a new Terminal. First you need to tell git your name, so that it can properly label the commits you make:

     git config --global user.name "Your Name Here"

Git also saves your email address into the commits you make. We use the email address to associate your commits with your GitHub account:

     git config --global user.email "your-email@youremail.com"


### GIT Tutorial
If you have never worked with git before, we recommend to go through the following basic git tutorial:

     http://excess.org/article/2008/07/ogre-git-tutorial/


## ROS - Robot Operating System
### Install ROS Electric
For the Fetch & Carry scenario the ROS distribution "Electric" is required. Other distributions have not been tested and therefore we do not recommend to use another distribution than "Electric". Follow the instructions on 

     http://www.ros.org/wiki/electric/Installation/Ubuntu

to install the basic ROS environment (make sure that you installed the ros-electric-desktop-full). Do not forget to update your .bashrc
  
Additionally to the ''ros-electric-desktop-full'' package, some few other packages are required to make the Fetch & Carry scenario compilable and executable. Therefore install the following packages:

     sudo apt-get install ros-electric-desktop-full ros-electric-arm-navigation ros-electric-pr2-controllers ros-electric-object-manipulation ros-electric-pr2-kinematics ros-electric-joystick-drivers ros-electric-laser-drivers ros-electric-cob-common  ros-electric-pr2-simulator ros-electric-openni-kinect ros-electric-pr2-apps ros-electric-bosch-drivers python-pygraphviz libmysqlclient-dev python-scipy libcap-dev bzr yaml-cpp0.2.6-dev ros-electric-orocos-toolchain ros-electric-rtt-ros-integration ros-electric-rtt-ros-comm ros-electric-rtt-common-msgs ros-electric-rtt-geometry libcap2-bin python-scipy python-setuptools

### ROS Tutorials
If you have never worked with ROS before, we recommend to go through the beginner tutorials provided by ROS:

     http://www.ros.org/wiki/ROS/Tutorials

In order to understand at least the different core components of ROS, you have to start from tutorial 1 ("Installing and Configuring Your ROS Environment") till tutorial 7 ("Understanding ROS Services and Parameters"). 


## Clone and compile the youBot Software
First of all you have to clone the youBot hackathon repository.

    cd ~
    git clone git@github.com:b-it-bots/hbrs-youbot-hackathon.git

Then go on with installing further external dependencies:

    sudo easy_install -U rosinstall vcstools

    cd ~/RoboCupAtWork
    rosinstall ../external_software repository.rosinstall
    rosdep install * -y

    cd ~/external_software/hbrs-youbot-hackathon-common
    rosinstall ../external_software repository.rosinstall

Now you have all the code which you need to operate/use the youBot platform. The new directory which you created in the beginning needs to be added the ROS_PACKAGE_PATH in your ".bashrc" in your home directory:

    echo "export ROS_PACKAGE_PATH=~/RoboCupAtWork:\$ROS_PACKAGE_PATH" >> ~/.bashrc
    echo "export ROS_PACKAGE_PATH=~/external_software:\$ROS_PACKAGE_PATH" >> ~/.bashrc
    source ~/.bashrc


Finally you should be able to compile the fetch & carry scenario:

    rosmake raw_fetch_and_carry                


If no errors appear everything is ready to use. Great job!


### Setting the Environment Variables
#### ROBOT variable
With the ROBOT variable you can choose which hardware configuration should be loaded when starting the robot. The following line needs to be added to the .bashrc:

     echo "export ROBOT=youbot-hbrs2" >> ~/.bashrc
     source ~/.bashrc



#### ROBOT_ENV Variable
The ROBOT_ENV variable can be used to switch between different environments. So please add the following line to your .bashrc:

     echo "export ROBOT_ENV=brsu-c025-arena" >> ~/.bashrc
     source ~/.bashrc



## Start e.g. a simple Fetch&Carry Scenario 
### In Simulation
    roslaunch raw_fetch_and_carry fetch_and_carry_demo_sim.launch

### At the Real Robot
    roslaunch raw_fetch_and_carry fetch_and_carry_demo.launch

### Execute the Scheduling Script
    rosrun raw_fetch_and_carry fetch_and_carry_demo.py
