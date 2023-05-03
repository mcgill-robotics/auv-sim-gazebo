# Clarke format explanation

This file is an in-depth description of how the clarke model works. I'll go line by line/block by block explaining what it is for. If there is something missing or incorret, 
please feel free to change (just make sure to ask someone to double check the changes you made).

For context - why SDF?
> SDFormat (Simulation Description Format), sometimes abbreviated as SDF, is an XML format that describes objects and environments for robot simulators, 
> visualization, and control. Originally developed as part of the Gazebo robot simulator, SDFormat was designed with scientific robot applications in mind. 
> Over the years, SDFormat has become a stable, robust, and extensible format capable of describing all aspects of robots, static and dynamic objects, lighting, terrain, and even physics.
```
<?xml version="1.0"?>
<sdf version="1.9">
```
At the beginning of each sdf file, you need to specify the version number. Make sure that all sdf files used in the same simulation have the same version.
The xml declaration is optional but recommended. 
