# Movement API Development Guide

There will be two main parts to develop for this system. The first system that
we will be updating is the Robot API (ROS Motor Code Drivers). The other system we will start
developing will be the Surface Robot modules.

## Robot API

From previous meetings, we have defined this layer as the exposed ports to interface
with our robot from an external source. This layer is a category of scripts that
manages the functionality of the robot. As of now, we have camera topics and motor
command topics. We will focus on the motor topics for this first set of development.

The first thing that I will update will be the script that translates wrench topics
to pwe values. Once that is set up, there should be no problem with running the
simulator and testing out the surface api.

The second thing that is perceived to be eventually be developed is the "smart navigation"
part of our system. In other words, the micro adjustments made to the PWEs to ensure
that we are heading in the direction of the commands. I imagine that we will want
the IMU sensor to make this functionality possible which means that it will need
to be developed before someone tackles this project.

1. Develop IMU Interface for easy access of IMU values
1. Develop smart navigation.

The final thing that we will be developing for this layer will be a smart decision
making system that will keep track of priorities for our commands. As of now it
isn't a priority due to the fact that we don't have our surface robot module ready
and we can't perceive how conflicting commands will need to be resolved but just
noting it because it is something that is to be done eventually.

## Surface Robot Module

The surface module is the big project for this year. We want a clean interface to
be able to manage robot movement while working with our autonomous code. The type
of functions we would want is from a simple Robot.setMove(force, torque) to a nicer
Robot.moveTo(dx, dy, dz) and Robot.rotate(angle).

In other words, the user of the module should not worry about any of the ROS nodes
and be able to comfortable assume that the robot will move based on the functions
that are provided.

The first part to develop will be to create our publishers and subscribers to manage
the data that will handle our desired topics. We have a system of publishers and
subscriber objects in our [surface server](https://github.com/uwrov/nautilus_surface/tree/main/src/uwrov_server/src)
directory. This work will essentially be separating our flask server from any
ROS dependencies and moving them into callable functions.

- Our goal for this section will be to just implement the Robot.setMove(force, torque)
method and a Robot.stop() method. After this has been finished, we will move onto
other functions outlined in the [Movement API](https://docs.google.com/document/d/1DrH8A5f9fOE4PFq12okoSF4D0etArVmkSRD1Vxxr4yY/edit). Note: Some of these functions may
require some information from the IMU and other sensors; make sure to have these
available before solving.

However, instead of relying on an object oriented paradigm, it may be easier to
move to a functional paradigm in which we instantiate the publisher/subscribers
within the module and only provide the functions for our module. This is completely
opinionated so if you are more comfortable with a different architecture, then
feel free to develop it as so. Just make sure to keep a consistent design.
