# Qualiying format explanation

This file is an in-depth description of how the Qualifying world works. I'll go line by line/block by block, explaining what it is for. If something is missing or incorrect, 
please feel free to change it (just ask someone to double-check the changes you made).

If you haven't read the Clarke description, I recommend reading that one first.

![first 2](https://user-images.githubusercontent.com/83185972/236067464-3b3480ee-6fab-4ad8-9887-62097fd90c76.gif)

## Context

World files should be used to organize all the models you created (in other files) and add plugins and physics to the simulation. This is why most of the file is composed of <plugin> and <include> tags.
  
> The world description file contains all the elements in a simulation, including robots, lights, sensors, and static objects. This file is formatted using SDF (Simulation Description Format), 
> and sometimes has a .world extension. The Gazebo reads this file to generate and populate a world.

## Code
  
### Plugins

The structure of a plugin here is the same as in Clarke. For more information on a specific plugin, you can check the following resource links:
* Physics system: [doc](https://gazebosim.org/api/gazebo/4.2/physics.html)
* User commands systems: [doc](https://gazebosim.org/api/gazebo/4.5/classignition_1_1gazebo_1_1systems_1_1UserCommands.html#details)
* Scene broadcaster system: [doc](https://gazebosim.org/api/gazebo/5.0/server_config.html)
* Contact system: [doc](https://gazebosim.org/api/gazebo/3.3/classignition_1_1gazebo_1_1systems_1_1Contact.html#details)
* Buoyancy system: [doc](https://gazebosim.org/api/gazebo/6.1/classignition_1_1gazebo_1_1systems_1_1Buoyancy.html)
* Particle emitter2 system: [doc](https://gazebosim.org/api/gazebo/5.1/particle_emitter.html)
  
### Scene

```
<scene>
    <ambient>1.0 1.0 1.0</ambient>
    <background>0.8 0.8 0.8</background>
    <grid>true</grid>
</scene>
```
The scene tag specifies the look of the environment (world).
* <ambient> is the colour of the ambient light
* <background> is for the colour of the background.
* <grid> if true, enables the grid visualization.

Documentation [link](http://sdformat.org/spec?ver=1.9&elem=scene)
  
### GUI

Set the properties and configurations of the User Interface when running the sim.
  
```
<plugin filename="MinimalScene" name="3D View">
    <ignition-gui>
        <title>3D View</title>
        <property type="bool" key="showTitleBar">false</property>
        <property type="string" key="state">docked</property>
    </ignition-gui>

    <engine>ogre2</engine>
    <scene>scene</scene>
    <ambient_light>0.4 0.4 0.4</ambient_light>
    <background_color>0.8 0.8 0.8</background_color>
    <camera_pose>-6 0 6 0 0.5 0</camera_pose>
</plugin>
```
  
Ignition GUI ships with the MinimalScene plugin, which instantiates a 3D scene and provides orbit controls but doesn't do much else. Actions such as adding, modifying and removing rendering elements from the scene must be performed by other plugins that work alongside the minimal scene.
  
Most of the tags are related to styling. Based on the type and name, you will be able to understand what it is for. For more information on the basic structure of a GUI, you can find the <gui> stag within the world [here](http://sdformat.org/spec?elem=world). 

<img width="715" alt="image" src="https://user-images.githubusercontent.com/83185972/236112704-439e9625-d962-4cbc-a699-db98fa6fae4a.png">

As you can see in the image, there is a grid (that we set to true in scene), and the colours match the values used.
  
Even if simple, I would like to highlight three essential plugins:
  
1) Image display:
```
<!-- Image display (from cameras) -->
<plugin filename="ImageDisplay" name="Image Display">
    <ignition-gui>
        <property key="state" type="string">docked</property>
    </ignition-gui>
</plugin>
```

<img width="273" alt="image" src="https://user-images.githubusercontent.com/83185972/236113000-a9ed2454-a910-4a9f-a1bb-22205a40ae90.png">
  
By using these values, we can see what the camera defined in clarke.sdf sees. Since we have more than one camera, we can reload (click on the arrow) and select the topic corresponding to the camera we want to see. It will then read the data from the topic and display it for the user. IMPORTANT: since it has to read from the topic, you need a flow of input to show something. So pausing the sim and changing the camera will NOT show anything; my advice => select the topic and click play and then pause. The reloading thing is only needed when you start the simulation.

2) Component Inspector:
  
```
<!-- Component Inspector -->
<plugin filename="ComponentInspector" name="Component inspector">
    <ignition-gui>
        <property type="string" key="state">docked</property>
    </ignition-gui>
</plugin>
```

<img width="276" alt="image" src="https://user-images.githubusercontent.com/83185972/236113862-f749d06d-a252-4c57-8e13-846d6a6aa2be.png">

This plugin is useful for seeing the configurations and plugins associated with the world. 

3) Entity Tree:
  
```
<!-- Entity tree -->
<plugin filename="EntityTree" name="Entity tree">
    <ignition-gui>
        <property type="string" key="state">docked</property>
    </ignition-gui>
</plugin>
```

<img width="201" alt="image" src="https://user-images.githubusercontent.com/83185972/236115056-b64a4142-dfb3-43da-bd8a-b55c5cd1fae2.png">
  
It displays a tree view with all the entities in the world. Useful as a way to quickly see the components of the world and what they are composed of.
  
### Include

```
<include>
    <uri>/home/felps/AUV-2023/catkin_ws/src/auv-sim/assets/quali/gate/gate.sdf</uri>
    <pose>-1 0 -1 0 0 0</pose>
</include>
```
  
As I'm sure you can see, there is a big problem with this code. Using the absolute path will not work on another computer. I'm having some difficulty finding a way to use relative path. SDF doesn't use the same $(find package) command as launch, and the [way to do it](https://answers.gazebosim.org//question/6568/uri-paths-to-packages-in-the-sdf-model-file/) (or at least the way I think to do it) is not working for me. If you want to give it a shot, I'll be very happy for the help. In the meantime, make sure to change the path to your computer before running.
  
That being said, let's go back to the explanations.
  
> Include resources from a URI. Included resources can only contain one 'model', 'light' or 'actor' element. The URI can point to a directory or a file. If the URI is a directory, it must conform to the model database structure. In other words, it includes the contents of other SDF files within the current SDF file. This allows for modular SDF files, where multiple SDF files can be combined to form a more complex robot or environment description.
  
* <uri> is the path to a resource.
* <pose> is the position of the resource in the world.
  
For more tag options, click [here](http://sdformat.org/spec?elem=world).
  
## You are ready to go!

Nice work reading everything!

![sen](https://user-images.githubusercontent.com/83185972/236116898-aebffe3e-5db1-4189-9c99-65eec283cd3f.gif)

Unless you didn't read everything and just skipped to the end. In that case...
  
![last 2](https://user-images.githubusercontent.com/83185972/236116401-c9ec6c95-06c3-44d4-abea-791ca549fa3b.gif)

  
