# Billy da Bebop - Autonomous Drone Demo
<img width=655 src="https://github.com/BosMathias/BosRepo/blob/fsm_operational/doc/Banner.png"  alt="billy da bebop"/>


Work in progress:
Master's thesis project by Rian Beck and Mathias Bos titled  
**'Interactive demo on the indoor localization, control and navigation of drones'**  
at KULeuven, Faculty of Engineering Science, Department of Mechanical Engineering  

This document serves as an instruction manual for installation and use of the demo.

## Hardware requirements
* Parrot Bebop 2 drone  
* HTC Vive (HMD, 2 Base Stations, 2 Controllers, Tracker + Dongle)
* Vive Tracker Mount for Parrot Bebop 2 (CAD files available [here](https://github.com/BosMathias/BosRepo/tree/fsm_operational/tracker%20mount))
* Gamepad (eg Logitech 710 or Xbox 360 controller) - not strictly necessary but advised for safety reasons

## Software requirements & setup
* Project developed in Ubuntu 16.04
* Python (developed with 2.7)
* ROS (developed with [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu))
* [OMG-tools](https://github.com/meco-group/omg-tools)
* [Steam](https://www.linuxbabe.com/desktop-linux/install-steam-ubuntu-16-04-lts-xenial-xerus), SteamVR

### Demo Setup
Browse to the *catkin workspace* and build with *catkin* to compile the code:
```bash
$ cd <path to BosRepo>/BosRepo/bebop_ws/
$ catkin clean
$ catkin build
```

### Steam Setup
To run the demo on a computer with a non-compatible graphics card for the HTC
Vive HMD, follow these instructions to construct a null-driver and use the HMD only as bluetooth dongle for the Vive controllers. Note that two bluetooth dongles as the one for the Vive tracker would work just as well, without any Vive HMD present.

The steps below are based on instructions found [here](https://gist.github.com/DanielArnett/c9a56c9c7cc0def20648480bca1f6772)
and [here](http://help.triadsemi.com/steamvr-tracking/steamvr-tracking-without-an-hmd).

Browse to following directory and open the default.vrsettings file with a text editor:

    $ cd <Steam Directory>/steamapps/common/SteamVR/drivers/null/resources/settings/
    $ gedit default.vrsettings

Set 'enable' to 'true'.  `<Steam Directory>` is probably `~/.local/share/Steam/` . Save this file and then browse to

    $ cd <Steam Directory>/steamapps/common/SteamVR/resources/settings/
    $ gedit default.vrsettings

Search for the “requireHmd” key under “steamvr”, set the value of this key to “false”.  
Search for the “forcedDriver” key under “steamvr”, set the value of this key to “null”.  
Search for the “activateMultipleDrivers” key under “steamvr”, set the value of this key to “true”.

Example default.vrsettings files are found [here](https://github.com/BosMathias/BosRepo/tree/fsm_operational/bebop_ws/src/vive_localization/vrsettings_example).

**Note -** Steam often reverses these changes during updates. Make sure to check these files when the system is not behaving as expected.

**Note -**  Even when the null driver is working properly, Steam might complain that the Vive Hmd is not found. The error 306 message can safely be ignored. Also the message in some versions of SteamVR that it is 'Not Ready' is normal. The demo will work just as well.


## Executing the demo
### Launch

* Make sure SteamVR is running and the Vive tracker as well as both Vive controllers are visible for the Vive lighthouses (fully green, not blinking in SteamVR window).  
* Make sure the Bebop drone is turned on and your computer is connected to it via Wi-Fi.
* Open a terminal, browse to the main folder of this repository `<path to BosRepo>/BosRepo/` and run the 'run_demo.sh' script:    


```bash
$ ./run_demo.sh
```
A new terminal window with five tabs will pop up. The tabs are running separately in order to separate useful outputs.  
The first tab is where the *roscore* is running. Usually no interesting output is found here. In the second tab there's *rviz* for visualization and in the third *rqt* for user input. Both *rviz* and *rqt* open their respective windows.  
To see the correct visualization in *rviz*, load the correct configuration file by clicking 'file', 'open config' and browse to `<path to BosRepo>/BosRepo/bebop_ws/src/configs/` and select 'demo.rviz'.  
To send commands (tasks) in rqt, load the correct configuration file by clicking 'perspectives', 'import...' and browse to `<path to BosRepo>/BosRepo/bebop_ws/src/configs/` and select 'demo.perspective'.

The fourth tab displays messages coming from the drone interface (Bebop Autonomy, see 'Acknowledgements' section).  

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

```
process[bebop_demo-1]: started with pid [8153]
process[controller-2]: started with pid [8154]
process[motionplanner-3]: started with pid [8155]
process[vive_localization-4]: started with pid [8156]
process[ctrl_keypress-5]: started with pid [8157]
 Waiting for Vive controllers ...
----    Bebop core running     ----
Building nlp ...  in 0.027586 s
----   Motionplanner running   ----
----    Controller running     ----
---- Vive Localization running ----
 Pull each trigger
```
If this is not the case, make sure to check for the common errors listed below.

The last message marked in blue instructs the user to identify the left and right Vive controller. The controllers only become active once both triggers have been pushed and the controllers are identified.  
The right controller is reserved for the demo operator. It is used for state transitions and actions that require a person standing in the flight area while the drone is flying. The left controller can be handed to a spectator because it is used only for operations during which the drone is at rest on the floor or that can be executed standing outside the flight area.

### Calibration
As default, the world reference frame with respect to the Vive Base Stations is calibrated for the setup in the Robot lab at the Department of Mechanical Engineering, KULeuven. To recalibrate, it suffices to edit the room dimensions in the 'controller.launch' file and publish once on the '/vive_localization/calibrate' topic via *rqt* or via the command line. The new world reference frame origin is the current location of the drone with the x-, y- and z-axs aligned with the drone's roll, pitch and yaw axis respectively. The origin is always defined as the center of the room on the ground.  
Keep in mind that the vive localization calibration will be overwritten at the next launch.

### Controller Tuning
All control parameters are set in `bebop_ws/src/bebop_demo/launch/controller.launch`.

### Executing Tasks
Now it's time for the actual demo. The demo is divided in a number of tasks: fixed sequences of states to illustrate specific control principles. The available tasks are:
* *'take-off'* and *'land'*  
Press the menu button on the right Vive controller. When taking off as part of the sequence of another task, do not use this button but press the trackpad on the right Vive controller to proceed.
* *'point-to-point'*  
Use OMG-tools to navigate from the current position to the goal evading the current set of (static) obstacles. The goal is set by the pulling the trigger on the right Vive controller and holding it at the desired location,
* *'draw follow traj'*  
Draw an arbitrary path using the left Vive controller. The drone flies to the starting point and subsequently tracks the drawn trajectory.
* *'drag drone'*  
Manipulate the drone as if he is fixed to the left Vive controller.
* *'undamped spring'* and *'viscous fluid'*  
Do positioning with only proportional gain or only derivative gain respectively, to show the analogy with an undamped spring or a viscous fluid.
* *'gamepad flying'*  
Manually fly around the drone using the gamepad.

Auxiliary tasks:
* *'place cyl obstacles'*  
Place cylindrical obstacles (room height, variable diameter) using the left Vive controller that are evaded in the 'point-to-point' task.
* *'place slalom obstacles'*  
Place obstacles to slalom between using the left Vive controller. Drag to define the side where the drone may not pass (only in y-direction). Execute 'point-to-point' task to perform the slalom.
* *'place plate obstacles'*  
Place plates that are evaded in the 'point-to-point' task using the left Vive controller.
* *'place hex obstacles'*  
Place hexagonal obstacles (variable height) that are evaded in the 'point-to-point' task using the left Vive controller.
* *'place window obstacles'*  
Place window obstacles (variable height and width) through which the drone flies in the 'point-to-point' task using the left Vive controller.

### Common Errors
* *No response from gamepad actions*.  
In the file `bebop_ws/src/bebop_autonomy/bebop_tools/launch/joy_teleop.launch` check the parameter "joy_dev". Your pc might see the gamepad as device "/dev/input/js1" while it should be '/js0' or vice versa.  
* *SteamVR doesn't find tracker or controllers*.  
Restart SteamVR, or Steam itself.  
Check your Steam 'default.vrsettings' files as discussed above in the Section 'Steam Setup'
* *Bebop drone takes off and immediately flies away into the surroundings*.  
Check the 'flat trim' of your drone. The config files from bebop_tools contain the button combination to set the flat trim (define level for your drone). This can alternatively be set via the [Parrot FreeFlight Pro](https://www.parrot.com/global/freeflight-pro#freeflight-pro-the-indispensable-application-to-pilot-your-drone) application on a smartphone.
* *Behavior that can not be explained by what is in the code*.  
Consider recompiling the code, using the commands mentioned under the Section 'Demo Setup' above. This is especially important when switching git branches.


## Acknowledgements
This project was built using a number of software packages for which we cannot claim credit. The sources can be found at following links:  
* [OMG-tools](https://github.com/meco-group/omg-tools)  
A Python software toolbox for spline-based optimal motion planning.

* [Bebop Autonomy](https://bebop-autonomy.readthedocs.io/en/latest/)  
A ROS interface for the Parrot Bebop 2 drone. This package provides an interface for decoupled x-, y- (roll and pitch angle) and z- (vertical velocity) input commands to the drone.  

* [Triad OpenVR](https://github.com/TriadSemi/triad_openvr)  
A python interface with the HTC Vive tracking system based on *pyopenvr*. We make the small addition in the use of the 'atan2' function rather than 'atan' for the calculation of Euler angles to enable four-quadrant operation.  
* [HTC Keypress](https://gist.github.com/awesomebytes/75daab3adb62b331f21ecf3a03b3ab46)  
Our keypress detection class is based on the example found in the link.
