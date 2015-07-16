# Development Environment Configuration
*How to configure a virtual machine to develop code for our 2014 - 2015 MATE ROV.*

##### Contents
* [Get an Ubuntu Image](#get-an-ubuntu-image)
* [Install VirtualBox](#install-virtualbox)
* [Create a Virtual Machine](#create-a-virtual-machine)
* [Configure the Virtual Machine](#configure-the-virtual-machine)
* [Install the Operating System](#install-the-operating-system)
* [Install VirtualBox Guest Additions](#install-virtualbox-guest-additions)
* [Install ROS](#install-ros)
* Install ROS Dependencies
    * [Joystick Drivers](#joystick-drivers)
* [Create a ROS Workspace](#create-a-ros-workspace)
* Install Required Packages
    * [OpenSSH Server](#openssh-server)
    * [Boost.Asio Serial Library](#boost.asio-serial-library)
    * [Joystick Utilities](#joystick-utilities)
    * [The Arduino IDE](#the-arduino-ide)
* [Configure Git](#configure-git)
* [Tidy Up the Launcher](#tidy-up-the-launcher)

##### Note
*When you see \<things-n-stuff\>, it's basically variable. Change it to match your system.*


## Get an Ubuntu Image:

*This download takes a bit, it's about a gig.*

Go to: http://www.ubuntu.com/download/desktop

Download: “64-bit – recommended”

## Install VirtualBox:

*The default options will work fine here.*

Go to: https://www.virtualbox.org/wiki/Downloads

Download: “VirtualBox 4.3.20 for \<your-operating-system> hosts”

## Create a Virtual Machine:

*You can name it \<whatever> you want.*

- Launch VirtualBox, click the blue “New” button.
- Name: \<whatever>. Type: “Linux”. Version: “Ubuntu (64 bit)”. Click “Next”.
- Set “Memory size” to “2048 MB” or something reasonable. Click “Next”.
- DEFAULT – Select “Create a virtual hard drive now”. Click “Create”.
- DEFAULT – Select “VDI (VirtualBox Disk Image)”. Click “Next”.
- Select “Fixed size”. Click “Next”.
- Set location to \<whatever> and size to “16.00 GB”. Click “Create”.
- Wait for the virtual drive to be created (about a minute or two).

## Configure the Virtual Machine:

- Click the yellow “Settings” button.
- Select “System” on the left, then “Processor” on the right.
  - Set “Processor(s):” to 2.
- Select “Display” on the left, then “Video” on the right.
  - Check “Enable 3D Acceleration”.
  - Set “Video Memory:” to “32 MB”. Click “OK”.
- Select “Network” on the left, then “Adapter 1” on the right.
  - Set “Attached to:” to “Bridged Adapter”.

## Install the Operating System:

- Click the green “Start” button.
- Select your .iso from the dropdown menu, or navigate to it using the folder icon. Click “Start”.

*If you get the error “FATAL: No bootable medium found! System halted.” Select “Devices” from the menu bar, hover “CD/DVD  Devices” and click on your .iso file or “Choose a virtual CD/DVD disk file…” and navigate to it. Select “Machine” from the menu bar and click “Reset”. Click “Reset” again.*

- Wait for the Virtual Machine to boot from the virtual disc. Click “Install”.
- Check “Download updates while installing” and “Install this third-party software”. Click “Continue”.
- Select “Erase disk and install Ubuntu” and check “Use LVM with the new Ubuntu installation”. Click “Install Now”.
- Type “Columbus” and select the appropriate location from the drop down, or don’t. Click “Continue”.
- DEFAULTS – Select “English (US)” for both, or some other language you prefer. Click “Continue”.
- Your name: \<anything>. Your computer’s name: \<hostname>. Pick a username: \<username>. -	Click “Continue”.
  - *Where \<username>@\<hostname>:~$ is how your command prompt will appear in your home directory.*
-	Make these simple. Remove “-VirtualBox” from the generated “Your computer’s name”.
-	Your password and whether it should be required for log in is up to you.
- Wait for Ubuntu to finish installing. This takes a few minutes. Grab a Coke.
  - When installation is complete, a windows will pop up. Click “Reset Now”.
- You should see the prompt “Please remove installation media and close the tray (if any) then press ENTER:”.
  - Press “ENTER”.
- You should now be able to log in. Try it.

## Install VirtualBox Guest Additions:

*This will let you resize the window in a reasonable manner.*

- Select “Devices” from the menu bar and click “Insert Guest Additions CD Image…”.
- Wait for the popup to appear. Click “Run”.
-	This button is likely off the screen. Drag the popup window to the left to see it.
- Enter your password. Click “Authenticate”.
-	This button is likely off the screen. Drag the popup window to the left to see it.
- Wait for the installation to complete. Press “Return”.
- Select “Machine” from the menu bar. Click on “Reset”.
-	The virtual machine should reboot and the window should now stretch reasonably.
- Log in. Right click the CD icon towards the bottom of the bar at the left. Click “Eject”.
-	This bar is the Launcher, it’s like the Start bar in Windows.

## Install ROS:

http://wiki.ros.org/indigo/Installation/Ubuntu

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xB01FA116
    sudo apt-get update
    sudo apt-get install ros-indigo-desktop-full

*This one takes a while, grab another Coke. Seriously though, this one’s long.*

    sudo rosdep init
    rosdep update
    echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    sudo apt-get install python-rosinstall

## Install ROS Dependencies

#### Joystick Drivers

http://wiki.ros.org/joystick_drivers?distro=indigo

    sudo apt-get install ros-indigo-joystick-drivers

## Create a ROS Workspace:

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

    mkdir –p ~/<catkin_ws>/src
    cd ~/<catkin_ws>/src/
    catkin_init_workspace
    cd ~/<catkin_ws>/
    catkin_make
    echo "source /home/<username>/<catkin_ws>/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

## Install Required Packages:

#### OpenSSH Server:

http://packages.ubuntu.com/trusty/openssh-server

    sudo apt-get install openssh-server

#### Boost.Asio Serial Library:

http://packages.ubuntu.com/trusty/libasio-dev

    sudo apt-get install libasio-dev

#### Joystick Utilities:

http://packages.ubuntu.com/trusty/joystick

    sudo apt-get install joystick

https://help.ubuntu.com/community/Sixaxis

    sudo apt-add-repository ppa:falk-t-j/qtsixa
    sudo apt-get update
    sudo apt-get install sixad

#### The Arduino IDE:

http://packages.ubuntu.com/trusty/arduino

    sudo apt-get install arduino
    arduino

*You’ll see a message about being added to the “dialout” group. Click “Add”. Enter your password. Don't close it yet...*

## Configure Git

http://git-scm.com/book/en/v2/Getting-Started-First-Time-Git-Setup

    $ git config --global user.name "John Doe"
    $ git config --global user.email johndoe@example.com

## Tidy Up the Launcher:

*You'll thank me later.*

- Right-click the Arduino IDE icon on the Launcher then click "Lock to Launcher".
  - You can close it now.
- Right-click the Terminal icon on the Launcher then click “Lock to Launcher”. 
  - You can close it now.
- For any unnecessary programs, right click and select “Unlock from Launcher”.
  -	LibreOffice Write, LibreOffice Calc, LibreOffice Impress, Amazon, etc.
