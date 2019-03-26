# Billy da Bebop - Autonomous Drone Demo
Work in progress:
Master's thesis project by Rian Beck and Mathias Bos titled  
**'Interactive demo on the indoor localization, control and navigation of drones'**  
at KULeuven, Faculty of Engineering Science, Department of Mechanical Engineering

## Hardware requirements
* Parrot Bebop 2 drone  
* HTC Vive (HMD, 2 Base Stations, 2 Controllers, Tracker + Dongle)
* Vive Tracker Mount for Parrot Bebop 2 (CAD files available [here](https://github.com/BosMathias/BosRepo))
* Gamepad (eg Logitech 710 or xbox 360 controller) - not strictly necessary but advised for safety reasons

## Software requirements & setup
* Project developed in Ubuntu 16.04
* ROS (developed with [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu))
* [OMG-tools](https://github.com/meco-group/omg-tools)
* [Steam](https://www.linuxbabe.com/desktop-linux/install-steam-ubuntu-16-04-lts-xenial-xerus), SteamVR 
* Python (developed with 2.7) + packages: numpy, scipy, fabulous

### Steam setup
To run the demo on a computer with a non-compatible graphics card for the HTC
Vive HMD, follow these instructions to construct a null-driver and use the HMD only as bluetooth dongle for the Vive Controllers. Note that two bluetooth dongles as the one for the Vive Tracker would work just as well, without any Vive Hmd present.

The steps below are based on instructions found [here](https://gist.github.com/DanielArnett/c9a56c9c7cc0def20648480bca1f6772)
and [here](http://help.triadsemi.com/steamvr-tracking/steamvr-tracking-without-an-hmd).

Browse to following directory and open the default.vrsettings file with a text editor:

    $ cd <Steam Directory>/steamapps/common/SteamVR/drivers/null/resources/settings/
    $ gedit default.vrsettings

Open it with a text editor to set 'enable' to 'true'.  `<Steam Directory>` is probably ~/.local/share/Steam/ . Save this file and then browse to

    $ cd <Steam Directory>/steamapps/common/SteamVR/resources/settings/
    $ gedit default.vrsettings

Search for the “requireHmd” key under “steamvr”, set the value of this key to “false”.  
Search for the “forcedDriver” key under “steamvr”, set the value of this key to “null”.  
Search for the “activateMultipleDrivers” key under “steamvr”, set the value of this key to “true”.

Example default.vrsettings are found [here](https://github.com/BosMathias/BosRepo/tree/fsm_operational/bebop_ws/src/vive_localization/vrsettings_example).

**Note -** Steam often reverses these changes during updates. Make sure to check these files when the system is not behaving as expected.

**Note -**  Even when the null driver is working properly, Steam might complain that the Vive Hmd is not found. The error 306 message can safely be ignored. Also the message in some versions of SteamVR that it is 'Not Ready' is normal. The demo will work just as well.


## Executing the demo
* Make sure SteamVR is running and the Vive Tracker as well as both Vive Controllers are visible for the Vive Lighthouses (fully green, not blinking in SteamVR window).  
* Make sure the Bebop drone is turned on and your computer is connected to it via Wi-Fi.
* Open a terminal, browse to the main folder of this repository `<path to BosRepo>/BosRepo/` and run the 'run_demo.sh' script:  


    $ ./run_demo.sh

A new terminal window with five tabs will pop up. The tabs are running separately in order to separate useful outputs.  
The first tab is where the *roscore* is running. Usually no interesting output is found here. In the second tab there's *rviz* for visualization and in the third *rqt* for user input. Both *rviz* and *rqt* open their respective windows.  
To see the correct visualization in *rviz*, load the correct configuration file by clicking 'file', 'open config' and browse to `<path to BosRepo>/BosRepo/bebop_ws/src/configs/` and select 'demo.rviz'.
To send commands (tasks) in rqt, load the correct configuration file by clicking 'perspectives', 'import...' and browse to `<path to BosRepo>/BosRepo/bebop_ws/src/configs/` and select 'demo.perspective'.

The fourth tab displays messages coming from the drone interface (Bebop Autonomy, see below).  

The fifth tab in the terminal contains the most interesting information for the demo user. Here all instructions, actions, state information and errors are displayed. It is advised to keep this tab open while running the demo.

Messages in this terminal follow the color code below:  

|  Color | Meaning  |
|--------|----------|
|<span style="color:green">  Green </span>|  Node is finished with configuration and is running properly
|<span style="color:cyan">  Cyan  </span>|  Message from Bebop Core
|<span style="color:yellow"> Yellow </span>|  Message from Controller
|<span style="color:magenta"> Magenta</span>|  Message from Motion Planner
|<span style="color:white"> White  </span>|  Message from HTC Keypress
|<span style="color:blue"> Blue   </span>|  Message from Vive Localization
|<span style="background-color:green;color:black"> Marked Green  </span>|  Instruction for the user
|<span style="background-color:blue;color:black"> Marked Blue  </span> |  Action from the user
|<span style="background-color:red;color:black"> Marked Red   </span> |  Error
|<span style="background-color:yellow;color:black"> Marked Yellow </span>|  Warning

If everything is working fine, the output below all ROS-node information and parameters should look like this:

```bash

```


Bebop Autonomy en Triad OpenVR en HTC Keypress credits  
rviz & rqt configs  
vive calibration!
klikken controllers, taak geven etc...

### Common errors
js0
steam default.vrsettings files
...

## Acknowledgements
