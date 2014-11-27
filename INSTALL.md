Get Ubuntu:

This download is about a gigabyte, start it first.

Go to: http://www.ubuntu.com/download/desktop

Download: “64-bit – recommended”


Install VirtualBox:

Go to: https://www.virtualbox.org/wiki/Downloads

Download: “VirtualBox 4.3.20 for <your operating system> hosts”

Install: The default options are fine. Maybe kibosh the shortcuts.

Create the Virtual Machine:

Launch VirtualBox, click the blue “New” button.

- Name: <whatever>. Type: “Linux”. Version: “Ubuntu (64 bit)”. Click “Next”.
- Set “Memory size” to “2048 MB” or more (if you can spare it). Click “Next”.
- DEFAULT – Select “Create a virtual hard drive now”. Click “Create”.
- DEFAULT – Select “VDI (VirtualBox Disk Image)”. Click “Next”.
- Select “Fixed size”. Click “Next”.
- Set name to “<whatever>” and size to “12.00 GB”. Click “Create”.
- Wait for the virtual drive to be created (a minute or two). You’re done.

Configure the Virtual Machine:

Click the yellow “Settings” button.

- Select “System” on the left, and “Processor” on the right. Set “Processor(s):” to 2.
- Select “Display” on the left, and “Video” on the right. Check “Enable 3D Acceleration”. Set “Video Memory:” to “32 MB”. Click “OK”.
- Select “Network” on the left, and “Adapter 1” on the right. Set “Attached to:” to “Bridged Adapter”.

Install the OS onto the Virtual Machine:

- Click the green “Start” button.
- Select your .iso from the dropdown menu, or navigate to it using the folder icon. Click “Start”.
- - - -	If you get the error “FATAL: No bootable medium found! System halted.” Select “Devices” from the menu bar, hover “CD/DVD Devices” and click on your .iso file or “Choose a virtual CD/DVD disk file…” and navigate to it. Select “Machine” from the menu bar and click “Reset”. Click “Reset” again.
- Wait for the Virtual Machine to boot from the virtual disc. Click “Install”.
- Check “Download updates while installing” and “Install this third-party software”. Click “Continue”.
- Select “Erase disk and install Ubuntu” and check “Use LVM with the new Ubuntu installation”. Click “Install Now”.
- Type “Columbus” and select the appropriate location from the drop down, or don’t. Click “Continue”.
- DEFAULTS – Select “English (US)” for both, or some other language you prefer. Click “Continue”.
- Your name: <anything>. Your computer’s name: <hostname>. Pick a username: <username>. -	Click “Continue”.
-	Where <username>@<hostname>:~$ is how your command prompt will appear in your home directory.
-	Make these simple. Remove “-VirtualBox” from the generated “Your computer’s name”.
-	Your password and whether it should be required for log in is up to you, just don’t encrypt it.
- Wait for Ubuntu to finish installing. This takes a few minutes. Grab a Coke.
- Click “Reset Now”.
- You should see the prompt “Please remove installation media and close the tray (if any) then press ENTER:”. Press “ENTER”.
- You should now be able to log in. Do it.

Install VirtualBox Guest Additions:

This will let you resize the window in a reasonable manner, and some other stuff.

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

Fix Your Launcher:

- Click the “swirly thing” at the top of the Launcher. Type “Terminal” click the “Terminal” icon.
- Right click the terminal icon at the bottom of the Launcher and click “Lock to Launcher”. 
- For any unnecessary programs, right click and select “Unlock from Launcher”.
-	LibreOffice Write, LibreOffice Calc, LibreOffice Impress, Amazon

Configure Apt:

- Open a Terminal window, type:
-	sudo apt-get update
-	sudo apt-get upgrade
- Type “y” when prompted for confirmation and hit enter. This will take a while. Time for another Coke.

Install Some Packages:

Install an SSH server.

http://packages.ubuntu.com/trusty/openssh-server
-	sudo apt-get install openssh-server
-	Press “y” then ENTER, wait a bit.

Install the Arduino IDE:

http://packages.ubuntu.com/trusty/arduino

-	sudo apt-get install arduino
-	Press “y” then ENTER, wait a bit.
-	Use the swirly search button in the Launcher to find “Arduino IDE” and click it.
-	You’ll see a message about being added to the “dialout” group, click “Add”.
-	Enter your password, right click the Arduino IDE icon on the Launcher and lock it.

Install Boost.Asio:

http://packages.ubuntu.com/trusty/libasio-dev

-	sudo apt-get install libasio-dev
-	Press “y” then ENTER, wait a bit.

Install PS3 Controller:

http://packages.ubuntu.com/trusty/joystick

-	sudo apt-get install joystick
- https://help.ubuntu.com/community/Sixaxis
-	sudo apt-add-repository ppa:falk-t-j/qtsixa
-	sudo apt-get update
-	sudo apt-get install sixad

Install Git:

http://packages.ubuntu.com/trusty/git

-	sudo apt-get install git


Install ROS:

http://wiki.ros.org/indigo/Installation/Ubuntu

-	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu trusty main" > /etc/apt/sources.list.d/ros-latest.list'
-	wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
-	sudo apt-get update
-	sudo apt-get install ros-indigo-desktop-full
•	This one takes a while, grab another Coke. Seriously though, this one’s long.

-	Set up ROS:
-	sudo rosdep init
-	rosdep update
-	echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc
-	source ~/.bashrc
•	sudo apt-get install python-rosinstall


Create ROS Workspace:

http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

-	mkdir –p ~/<catkin_ws>/src
-	cd ~/<catkin_ws>/src/
-	catkin_init_workspace
-	cd ~/<catkin_ws>/
-	catkin_make
-	source devel/setup.bash
