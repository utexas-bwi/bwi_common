BWI Virtour
===========

Virtour is a public facing system for teleoperated building tours.

* Web address: http://www.cs.utexas.edu/~larg/bwi_virtour/
* On the robot: uses ROS to manage tour, stream video, and take commands.
* On the website: uses rosjs to pipe commands to the robot

## Features

* Adaptive Video streaming
* Teleoperated servo control
* Goal based logical navigation
* In-place robot rotation
* Robot position marker
* Tour leader management

## Instructions

On robot:

1. Set up smallDNS

2. Change `broadcastTour.sh` to email correct people and with correct url

2. Bring up robot (eg: `roslaunch bwi_launch segbot_v2.launch`)

3. Launch `roslaunch bwi_virtour virtour_passive.launch`

4. Launch `roslaunch bwi_virtour virtour_active-[cameratype].launch`


On server:

1. Link or copy the `web` folder to `/var/www`

2. Modify `index.html` to add robot

## Launch Files

There are two types of launch files, passive and full.

Before executing the virtour launch files make sure that the segbot launch
files were executed (either `segbot_v1.launch` or `segbot_v2.launch`).

The passive launch files brings up the tour manager, rosbridge and mjpeg
servers. Allows people to view the robot's position, camera feed, and scavenger
hunt status.

The full (active) launch files (no longer requires the passive one to be
running), allows people to become tour leaders, control rotation, request
navigation, as well all the features of passive.

Note that at the launch files assume you are running on a kinect powered robot.
Comments in the launch file will provide a working launch file if servo or
point-grey camera support is needed.

## ROS Nodes

### Tour Manager

Handles by `tour_manager`. Manages the tour state machine. Handles tour leaders, when
tours are allowed, and authentication.

#### Exposed Services

* `/tour_manager/authenticate` - used to authenticate users and check if they are leader
* `/tour_manager/get_tour_state` - get the state of the current tour (excluding leader hash)
* `/tour_manager/leave_tour` - used to demote leaders
* `/tour_manager/ping_tour` - used to keep the tour leader alive
* `/tour_manager/request_tour` - used to request tours

### Logical Navigation

Handled by the `go_to_location_service_node`.
To add new goals, edit `js/virtour.js`

#### Exposed Services

* `/go_to_location` - used to navigate to semantic locations

### Rotation

Handled by `rotation_service_node`. Takes in a float for the rotation delta

#### Exposed Services

* `/rotate` - used to perform on-the-spot rotations

## Project structure

* `/web` contains all files necessary for hosting the website
* `/src` contains the c++ source for the ROS nodes
* `/srv` contains the service definitions


## Resources

[Old Talk](https://docs.google.com/presentation/d/1cNeUuevuT522KYIJN_F945JEqcbchYooTK5Sci0EaNs/edit?usp=sharing)

[Small DNS](https://github.com/pato/smallDNS)
