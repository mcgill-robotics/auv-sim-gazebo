# Clarke format explanation

This file is an in-depth description of how the Clarke model works. I'll go line by line/block by block, explaining what it is for. If something is missing or incorrect, 
please feel free to change it (just ask someone to double-check the changes you made).

## For context 
1) Why SDF?
> SDFormat (Simulation Description Format), sometimes abbreviated as SDF, is an XML format that describes objects and environments for robot simulators, 
> visualization, and control. Originally developed as part of the Gazebo robot simulator, SDFormat was designed with scientific robot applications in mind. 
> Over the years, SDFormat has become a stable, robust, and extensible format capable of describing all aspects of robots, static and dynamic objects, lighting, 
> terrain, and even physics.
2) What is XML?
> XML (Extensible Markup Language) is a markup language used to store and transport data in a structured format. 
> It is a text-based format, which means that XML documents can be read and edited by humans as well as machines. It is a general-purpose language that can 
> be used to represent any kind of data in a structured format, making it useful for a wide range of applications.

## Let's look at the code
#### Config
```
<?xml version="1.0"?>
<sdf version="1.9">
```
At the beginning of each sdf file, you need to specify the version number. Make sure that all sdf files used in the same simulation have the same version.
The xml declaration is optional but recommended. 

#### Model
```
<model name="clarke">
</model>
```
The model tag is where we define the robot or any physical object. This is where we specify all the attributes, features, sensors, etc., related directly related 
to the robot. From personal experience, you should aim to choose a short and accurate name for the model. It will be later used for naming topics, so if it is 
too long or too weird, it will be a pain in the ass to write the name or remember what object it refers to. I know it sounds silly, but when you are stressed because ignition
gazebo absolutely sucks, the last thing you need is to throw more fuel in the fire.

Important: An SDFormat file can consist of just a model. A world can contain many models (if you don't know what worlds are yet, think of it as the world inside the simulation).

#### Link
```
<link name="base_link">
</link>
```
A link refers to a rigid body in the robot or environment model. It contains the physical properties of one body of the model. This can be a wheel or a link in a joint 
chain. In our case, what we define by "base_link" is the base of the robot or, in other words, the main body. Each link may contain many collision and visual elements. 
For performance reasons, it is good to not exaggerate the number of links in a single model.

For Clarke, we decided to have a link for the main body and an additional link for each thruster.

To see all the possible physical properties you can assign to a link, I recommend checking the official
[SDFormat Specification website](http://sdformat.org/spec?ver=1.9&elem=link "Link Specifications").

#### Inertial
```
<inertial>
  <mass>147.8671</mass>
  <inertia>
    <ixx>3.000000</ixx>
    <ixy>0</ixy>
    <ixz>0</ixz>
    <iyy>41.980233</iyy>
    <iyz>0</iyz>
    <izz>41.980233</izz>
  </inertia>
</inertial>
```
This part is a little bit tricky since it requires some math and physics. The inertial tag is used to specify the link's mass (kg), position of its center of mass,
and moments of inertia about its principal axes. The first two concepts are easy to understand, so I won't bother explaining. If you don't know what the moment of inertia
is, I would recommend [this](https://en.wikipedia.org/wiki/Moment_of_inertia# "Moment of Inertia Wiki") as a quick introduction. To calculate the values for the inertia tag, you
should look into [inertia tensor](https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor "Inertia Tensor Wiki"). Note that we only enter six out of nine matrix values 
because the matrix is symmetrical.

#### Collision
```
<collision name="main_body_collision">
  <geometry>
    <cylinder>
      <radius>0.2</radius>
      <length>0.788</length>
    </cylinder>
  </geometry>
</collision>
```
Let's come back to something more intuitive. The collision defines, well, the collision properties of the link. Not much surprise there. Some of the tags worth highlighting here are:
pose, geometry, and surface. For Clarke, we didn't include the pose tag because it has a default value <pose>0 0 0 0 0 0</pose> in relation to the link. If you are unsure what
these values mean, the first three numbers are (x, y, z) and the last three (roll, pitch, yaw), and the default measurement units are meters and radians. The geometry is simply the 
shape. There are multiple [options for the types of shapes](http://sdformat.org/spec?ver=1.9&elem=geometry "<geometry> Specifications"), such as box, cylinder, capsule, etc.

#### Visual
```
<visual name="visual">
  <geometry>
    <cylinder>
      <radius>0.2</radius>
      <length>0.788</length>
    </cylinder>
  </geometry>
</visual>
```
A visual element is used to visualize parts of a link. Since this is a simple object, we used the same values as in collision. The visual tag is not that important
to the functionality of the simulation, so I won't waste a lot of time on it. However, it does look better when you put a little bit more "life" into the sim. Up to you.
Here is the [specifications](http://sdformat.org/spec?ver=1.9&elem=visual "bro, are you sure?... might as well do a fashion week event at this point") if you want to learn more.

#### IMU sensor
```
<sensor name="imu_sensor" type="imu">
  <always_on>1</always_on>
  <update_rate>7</update_rate> 
  <visualize>true</visualize>
  <topic>imu</topic>
</sensor>
```
Now we are getting into the more interesting but also frustrating part. When it comes to sensors, the documentation out there is shit, and the examples are even worse. 
I recommend five minutes of meditation before any programming related to sensors.

Ok, so luckily, IMU is very chill. Ignition gazebo already has the plugin designed. The only thing you have to worry about is getting the sensor type right and setting the 
properties value. Let me also give an important spoiler: whenever you add a sensor, you need to add the corresponding plugin. If you scroll down to near the bottom of the
file, you will see a bunch of plugins. The first one is for the IMU.
```
<plugin 
    filename="libignition-gazebo-imu-system.so"
    name="ignition::gazebo::systems::Imu">
</plugin>
```
We will come back to the plugin in a minute. Let's first finish the sensor tag. 
* `<always_on>` if true, the sensor will always be updated according to the update rate.
* `<update_rate>` is the frequency at which the sensor data is generated (updates per second).
* `<visualize>` if true, the sensor is visualized in the GUI. You can't see in the Clarke because the positioning of the sensor is inside the main body. It is set to true
just for debugging if needed.
* `<topic>` is the name of the topic on which data is published.

I highly recommend checking this [github page](https://github.com/gazebosim/docs/blob/master/dome/sensors.md "IMU Sensor") for more examples.

#### Cameras
We can also add cameras to the model and visualize it in the GUI. It's really cool to see from the perspective of the model. Anyway, we have three cameras in total: one below and two front.
```
<sensor name="down_camera" type="camera">
  <horizontal_fov>1.047</horizontal_fov> <!-- not actual value -->
  <update_rate>30.0</update_rate>
  <visualize>true</visualize>
  <topic>down_cam</topic>
  <always_on>1</always_on>
  <image>
    <width>800</width>
    <height>800</height>
  </image>
  <noise>
    <type>gaussian</type>
    <mean>0.0</mean>
    <stddev>0.007</stddev>
  </noise>
</sensor>
```
As you have probably recognized by this point, some properties are the same for different tags. Whenever that happens, I will only re-explain it if there is something
unique about it in the specific context.
* `<horizontal_fov>` determines the camera's field of vision. You should check the camera's properties on the real Clarke before adjusting this value.
* `<image>` determines the properties of the images captured. Again, check the properties of your camera.
* `<clip>` you can determine how far and close the camera can see. It's a useful feature since a real camera can't see everything in its direction, especially underwater.
* `<noise>` this tag is up to you. Noise is sampled independently per pixel on each frame. That pixel's noise value is added to each of its color channels, 
which at that point lie in the range [0,1]. I would say it makes the model a bit more realistic, but if you set the values too high, it becomes a headache.






