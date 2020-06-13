# Neato XV ROS
This repository holds a complete stack of ROS packages intended to be used along with a *hacked* unit of Neato XV 11/14 vacuum cleaner.  
By hacked, it means that the unit was completely stripped off its insides (except for the lidar unit) so that it provides a differential-drive platform featuring a base, a lidar, 2 wheels, 2 motors and 2 encoders.
The robot's commanding unit had been replaced with a Raspberry Pi 3, running Ubuntu Mate 18.04 along with ROS melodic.  
In general, many scripts or launch files are separated into a "simulation" version running on Gazebo, and a "real" version running on the actual platform.  
While the real version requires the actual augmented neato platform with all of its electronics and wiring, the simulation can be used on any PC as a means of tuning parameters (depending on the fidelity of the real Neato) or just playing around.
**The code is presented "as-is", without warranty of any kind, including but not limited to direct, indirect or implied damage or loss of profit. Use it at your own risk.**

# Prerequisites
In order to fully use this stack, one must obtain and/or install several things, depending on the mode which will run:
## Simulation:
One only needs to have a PC (can't point to minimum specs, but any intel gen.5 processor and higher should do, preferrably with any Nvidia graphics card), with the following installed:
1. Ubuntu 18.04  (might work on higher versions).
2. ROS melodic  (if a newer version of Ubuntu is installed, a matching ROS distro must be used).
3. ROS navigation stack (sudo apt install ros-distro-navigation).
4. Since the turtlebot_teleop_key node is used, and is maintained only up until ROS Kinetic, its source script file is included in this stack.

## Real:
One must build the Neato according to instructions. Hardware and software requirements are:
1. Obviously, a Neato XV vacuum cleaner platform (stripped down), with its 2 original batteries.
2. Raspberry Pi 3.
3. 5v/2A capable power bank (preferrably of a small form-factor). I used a Miracase 5000mAh power bank.
4. Rpi motor driver shield, I used the following model: https://www.waveshare.com/wiki/RPi_Motor_Driver_Board  (can be replaced with any dual-motor H-bridge motor driver, but may require a hardcoded change on your side, like pin numbers, etc).
5. USB-to-TTL converter / FTDI cable for communicating with the lidar.
6. A driver for sniffing the lidar output **has already been included here**, special thanks to rohbotics: https://github.com/rohbotics/xv_11_laser_driver/tree/kinetic-devel/include/xv_11_laser_driver
7. N-channel MOSFET or an additional motor driver for spinning the lidar motor.
8. pigpio library (pip install pigpio).
8. Optional (but recommended): A remote-control with a 4-channel (or more) receiver binded to it (basically, any drone or car RC). This can be used to easily drive the Neato around.

# Content Overview

## Robot Description
The neato_description package holds UDRF (more percisely SDF) files, describing the TF relationships between the robot's links, including some physical characteristics for use with Gazebo simulator.
It also holds some meshes for the various parts so that the Neato can be *roughly* visualized.
The SDF contains an extra link and sensor for an RGBD camera (Kinect / Astra etc.) which was deprecated in the real version but is usable through the simulation.
**Note**: The dimensions portrayed both within the URDF files and in the meshes are approximate (measured using a ruler), and surely induce errors within the full system to some extent. Even so, it does work not too bad though.
The main difference between the simulation/real description files is the name of the LaserScan topic, so that Rviz, Gazebo and real-lidar definitions match each other in the given mode.

## RC Teleoperation
The teleop_rc package holds a python module which enables the user to read standard RC receiver inputs with almost any pin on the Raspberry Pi.
It also contains a ROS node which utilizes this module to read the said signals and publishes a topic with the read inputs.
By "standard receiver inputs", I mean 50Hz (20 ms sample time) PPM signals, with duty cycles ranging between 1000-2000 microseconds.
The type returned by the function is an integer depicting microseconds in which the signal was HIGH during the sample time, and can actually have values lower or greater than 1000-2000, should the "high-time" be out of this range. 
The code uses interrupts, and even with Ubuntu not being a real-time OS, the returned values are stable and accurate as far as I have tested.
**It can be utilized in any project in need of this input, and you are most welcome to do so as long as you know what you are doing, without any warranty**.

## Simulation
This package only holds launch files adapted for using the simulation mode. They are separated in order to enable inclusion of individual components inside several other launch files.
In general, they allow the user to spawn the robot within a Gazebo environment, choosing between the willow_garage offices or a representation of my old living room (on the obstacle level).
Other environments will have to be created by the user according to their need.
The SDF loads the robot TF, meshes and collisions, a differential drive plugin, and publishes sensor output topics (1-channel lidar and an RGBD camera).
For more details, see the "How to use" section below.

## Navigation
Holds any package or node which has anything to do with moving the Neato using the PC. 
In particular, it governs the definitions for the custom Gazebo world(s), reference 2D maps and ROS navigation stack's parameter yaml files.
The Neato can either be using the keyboard or a remote controller, OR be given a 2D nav goal via Rviz and let the navigation stack move it there by itself, avoiding obstacles, etc.
**Note that there is a vast amount of parameters and they have only been loosely calibrated. Fine-tuning might be necessary to get use-case specific results.**

## Real Neato
Contains all that is needed to control a real-life Neato platform that had been augmented with a ROS-installed Raspberry Pi.
Besides having the necessary launch files, it contains some python scripts:
1. Basic PIDF control module with velocity filtering and reset-windup mitigation. You may choose to use either velocity or position control, but it is not recommended to switch between them per motor in the same run.
2. An interrupt-based module capable of reading encoders of various kinds (1-channel or 2-channels with regular/double/quadrature resolution) using almost any pin.  
Unfortunately, the Neato's encoders are of 1-channel type which lowers the accuracy of readings, especially when the wheel changes direction.  
Note that the 2 modules above can be used in any project utilizing encoders and/or closed-loop feedback control system (not only motors), **given that you know what you are doing. No warranties whatsoever**.
3. Differential-driver node, using the above modules to turn velocity input commands into robot movement and publish an odometry topic + TF.
4. Command-handling node, subscribing to inputs from several sources and publishing only he desired ones to the robot (differential driver node).
5. Node for rotating the lidar motor itself. The lidar spits out data only when it is turning at a speed of 300RPM +-5%. Must be used in conjuction with the provided xv_11_laser_driver node and might need tuning (lidar units leave the manufacturer with different frictions). 


# How to use
Understand that this is not a tutorial about how to use the navigation stack.
For more information about the works of move_base, costmaps, planners, amcl and others, please refer to ROS's navigation stack documentation.

## Simulation
This mode does not require connecting to the Neato, since the Rpi is not even in the loop.  
If you want to drive the Neato around the simulated Gazebo world using your keyboard, you can either launch standalone_gazebo_living_room.launch or standalone_gazebo_willow_garage.launch.  
If you installed everything correctly, it should all open up, including Rviz subscribing to the correct topics.  
**This will NOT include navigation commands**.
To make the robot respond to the keyboard, you must have the launched terminal window active. For instructions and more information, please refer to turtlebot_teleop or teleop_twist_keyboard documentation.

If you want to use the navigation stack, launch neato_simulation_navigation.launch with several arguments:
1. "amcl" - defaults to false. amcl is in charge of dynamic localization of the robot, matching between the laserscan topic and the world's underlying 2D map (instead of relying on dead-reckoning for odometry).
2. "living_room" and "willow_garage" - decides which world to load. **While one is set to true, the other one should be set to false, or else it would not work right**.
Defaults to "living_room" being true and "willow_garage" being false, since the living room is a lighter world to load.

## Real Neato
This requires both a PC and the Rpi to work together, since the Rpi is not powerful enough to make the calculations necessary for navigation. This means that you will need to do some touch-ups for your Rpi.

### Pre-setup
1. Verify USB-converter device for the lidar. 
Use your preferred method to check the FTDI (or equivalent) device name. It should probably default to /dev/ttyUSB0 if nothing else is connected to the USB ports.
You may do so, for example, by opening a terminal and typing ls /dev, then connecting the device, running the same command again and look for any differences.
Once you found the device name, open the file "neato_lidar_drivers.launch" and make sure that the device name stated there matches the one you found.
Now you must ensure that you have permissions to use this device. Open a terminal and run the command (if your device is different, use its name instead):
sudo chmod 777 /dev/ttyUSB0  
**You must run this every time you restart you Rpi**, so you may want to add an alias to your bashrc.
There is also a way to make this permission permanent by adding your user to the device group permissions, please refer to google about how it's done.  
**One more thing - in my setup, the device would only work correctly if the Rpi boots up with the USB device already connected. 
If the device is connected after boot-up it would read garbage RPMs from the lidar, rendering the rotation control node useless. Took me quite a while to realize this!**

2. Measuring RC ranges. 
If you decide to use an RC for movement, you should first meaure the maximums and minimums of your 3 channels so that you can feed those into the controller.
Choose your RC channels such that at least one of them does NOT "bounce back" to zero, like the throttle stick or a general switch on the RC. 
This "choosing" channel decides whether the robot input will be taken from the RC or the navigation. Yes, the RC decides wheather to switch from manual control to navigation control and vice versa.  
After this, you should:  
A. Connect it as described in the connection scheme.  
B. Open the "ppm_input_rpi.py" file, move to the bottom and place the correct pin numbers within the list **in BCM format**.  
C. Run that script using regular python 2.7 .  
D. The script will print the channel outputs on the screen. Move the control sticks you want to use to their edges in every direction a few times.  
E. Make sure that the "choosing" axis is channel 0, "forward-backward" axis is channel 1 and "rotate-left-right" axis is channel 2.  
F. Exit the script and insert the printed values appropriately inside the file "command_handler.py".  

3. Setup a remote master-slave network.  
If you don't want to connect the Rpi to a screen and keybboard every time you use it, you need to make sure the ROS master and slave are aware of each other.
You can choose either one to be the master, but I like having the PC as it takes processing off the Rpi.  
To do so, follow these high-level instructions:  
A. Pick and set a known static IP for both the PC and the Rpi on the network you will be using.  
B. In the Rpi, set a new host named "master" in /etc/hosts, having the PC static IP.  
C. Do the same in the PC, but for a host named "neato" having the Rpi IP.  
D. In the Rpi, export an environment variable named "ROS_IP" **containing the Rpi's own IP**.  
E. Do the same in the PC **with the PC's own IP**.  
F. **Only on the Rpi**, export an environment variable named "ROS_MASTER_URI", containing the string "http://master:11311"

This will make sure bi-directional communication is possible between the two. Some people say it works with only some of these actions, for me it only worked flawlessly using all.  
It can't harm, so just do yourself a favor and follow all of them. It is also recommended to place some of these as aliases, and also make sure Rpi connects to the desired Wifi network upon startup.

### Startup workflow. 
After the connection had been set up, you should boot up your PC and Rpi, and within the PC:
1. Open a terminal window, start a roscore and minimize it.
2. Open another terminal, and launch the file "neato_real_full.launch". It can receive an argument "navigation" as true or false (defaults to true). Setting it to false will only allow you to drive the neato using the remote or the keyboard.  
If an RC is NOT present, then the Neato would react either to the navigation commands or the keyboard, according to your choice of the "navigation" argument value.  
if an RC IS present, then when the "choosing channel" is low (<1400 ms) the Neato would be driven using the other RC channels, otherwise it will act as in the above statement.
3. SSH into your Rpi (remember, it is saved as a host named neato) and run the pigpiod daemon, device permissions as stated above, and export needed variables.
4. None of the statements in (3) are blocking, so you should be able to use the same terminal window to launch the "neato_controller_with_lidar.launch" file.  
If you do not wish the lidar to be active, you can launch "neato_base_controller.launch" but then obstable avoidance and amcl would not work.
5. The TF tree publication should now be complete using both PC and Rpi, and the Rviz (in the PC) should show everything needed and also allow setting nav goals for the Neato.

### Code workflow. 
Just for reference, the workflow in the full navigation mode goes as follows:
1. Rpi receives lidar data and publishes it as a LaserScan topic.
2. PC picks up the LaserScan messages and calculates costmaps.
3. User defines a destination pose using Rviz's 2D nav goal button in the top toolbar.
4. PC generates a path to that pose, and starts sending messages on a /cmd_vel topic.
5. Rpi picks up these messages and controls the Neato wheels to fulfill these velocity commands.
6. Repeat.

