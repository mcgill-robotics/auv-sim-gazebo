# Qualiying format explanation

This file is an in-depth description of how the Qualifying world works. I'll go line by line/block by block, explaining what it is for. If something is missing or incorrect, 
please feel free to change it (just ask someone to double-check the changes you made).

If you haven't read the clarke description, I recommend reading that one first.

![first 2](https://user-images.githubusercontent.com/83185972/236067464-3b3480ee-6fab-4ad8-9887-62097fd90c76.gif)

## Context

Ideally, world files should be used as a way to organize all the models you created (in other files) and add plugins and physics to the simulation. This is why most of the file is composed of <plugin> and <include>.
  
> The world description file contains all the elements in a simulation, including robots, lights, sensors, and static objects. This file is formatted using SDF (Simulation Description Format), 
> and sometimes has a .world extension. The Gazebo reads this file to generate and populate a world.
  
### Plugins

The structure of a plugin here is the same as in Clarke. For more information on a specific plugin, you can check the following resource links:
* Physics system: [doc](https://gazebosim.org/api/gazebo/4.2/physics.html)
* User commands systems: [doc](https://gazebosim.org/api/gazebo/4.5/classignition_1_1gazebo_1_1systems_1_1UserCommands.html#details)
* Scene broadcaster system: [doc](https://gazebosim.org/api/gazebo/5.0/server_config.html)
* Contact system: [doc](https://gazebosim.org/api/gazebo/3.3/classignition_1_1gazebo_1_1systems_1_1Contact.html#details)
* Buoyancy system: [doc](https://gazebosim.org/api/gazebo/6.1/classignition_1_1gazebo_1_1systems_1_1Buoyancy.html)
* Particle emitter2 system: [doc](https://gazebosim.org/api/gazebo/5.1/particle_emitter.html)
  
### GUI

