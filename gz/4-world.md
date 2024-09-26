# Building Simulation Worlds with SDF

In this tutorial, we'll learn how to create a simulation world using the **Simulation Description Format (SDF)** and how to add models to it.

## Defining the World

Every SDF world begins with the following structure:

```xml
<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="world_demo">
    <!-- World content goes here -->
  </world>
</sdf>
```

- The first line specifies the **XML version**.
- The `<sdf>` tag defines the **SDF version** being used (`1.8` in this case).
- The `<world>` tag names the world (`world_demo`) and encloses all the elements that define the simulation environment.

## Setting Up Physics

Define the physics properties of the world using the `<physics>` tag:

```xml
<physics name="1ms" type="ignored">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

- **`name="1ms"`**: An arbitrary name indicating a time step of 1 millisecond.
- **`type="ignored"`**: Specifies the physics engine type. Although options like `ode`, `bullet`, `simbody`, and `dart` exist, setting it to `"ignored"` defers the selection to the simulator's default.
- **`<max_step_size>`**: The maximum time step for each simulation iteration (`0.001` seconds). A smaller value increases accuracy but requires more computational power.
- **`<real_time_factor>`**: The ratio of simulation time to real time (`1.0` means real-time simulation).

## Adding Plugins

Plugins are dynamically loaded pieces of code that extend the functionality of the simulation. Here are some essential plugins:

### Physics Plugin

```xml
<plugin
  filename="gz-sim-physics-system"
  name="gz::sim::systems::Physics">
</plugin>
```

- This plugin is crucial for simulating the dynamics of the world.

### User Commands Plugin

```xml
<plugin
  filename="gz-sim-user-commands-system"
  name="gz::sim::systems::UserCommands">
</plugin>
```

- Enables user interactions such as creating, moving, and deleting models within the simulation.

### Scene Broadcaster Plugin

```xml
<plugin
  filename="gz-sim-scene-broadcaster-system"
  name="gz::sim::systems::SceneBroadcaster">
</plugin>
```

- Handles the broadcasting of the world scene for visualization purposes.

## Configuring the GUI

The `<gui>` tag defines elements related to the Gazebo graphical user interface (GUI):

```xml
<gui fullscreen="0">
  <!-- GUI plugins and configurations -->
</gui>
```

Gazebo GUI offers a variety of plugins to enhance the user experience. We'll add essential plugins to get our world running with basic functionality.

### 3D Scene Display

```xml
<!-- 3D Scene -->
<plugin filename="MinimalScene" name="3D View">
  <gz-gui>
    <title>3D View</title>
    <property type="bool" key="showTitleBar">false</property>
    <property type="string" key="state">docked</property>
  </gz-gui>

  <engine>ogre2</engine>
  <scene>scene</scene>
  <ambient_light>0.4 0.4 0.4</ambient_light>
  <background_color>0.8 0.8 0.8</background_color>
  <camera_pose>-6 0 6 0 0.5 0</camera_pose>
  <camera_clip>
    <near>0.25</near>
    <far>25000</far>
  </camera_clip>
</plugin>
```

- **Properties**:
  - **`showTitleBar`**: If set to `true`, displays the plugin's title bar.
  - **`state`**: Determines the plugin's position (`"docked"` or `"floating"`).
- **Rendering Engine**: Choose between `ogre` and `ogre2` (we're using `ogre2`).
- **Lighting and Background**:
  - **`<ambient_light>`**: Sets the ambient light color.
  - **`<background_color>`**: Sets the background color of the scene.
- **Camera Configuration**:
  - **`<camera_pose>`**: Defines the camera's position (`X Y Z`) and orientation (`Roll Pitch Yaw`).
  - **`<camera_clip>`**: Sets the near and far clipping planes for the camera.

### Scene Manager Plugin

```xml
<plugin filename="GzSceneManager" name="Scene Manager">
  <gz-gui>
    <property key="resizable" type="bool">false</property>
    <property key="width" type="double">5</property>
    <property key="height" type="double">5</property>
    <property key="state" type="string">floating</property>
    <property key="showTitleBar" type="bool">false</property>
  </gz-gui>
</plugin>
```

- Manages the scene elements and provides options to manipulate them.

### World Control Plugin

```xml
<!-- World Control -->
<plugin filename="WorldControl" name="World Control">
  <gz-gui>
    <title>World Control</title>
    <property type="bool" key="showTitleBar">false</property>
    <property type="bool" key="resizable">false</property>
    <property type="double" key="height">72</property>
    <property type="double" key="width">121</property>
    <property type="double" key="z">1</property>
    <property type="string" key="state">floating</property>
    <anchors target="3D View">
      <line own="left" target="left"/>
      <line own="bottom" target="bottom"/>
    </anchors>
  </gz-gui>

  <play_pause>true</play_pause>
  <step>true</step>
  <start_paused>true</start_paused>
  <service>/world/world_demo/control</service>
  <stats_topic>/world/world_demo/stats</stats_topic>
</plugin>
```

- **Functionality**:
  - Provides controls to play, pause, and step through the simulation.
  - **`<start_paused>`**: If set to `true`, the simulation starts in a paused state.
- **Properties**:
  - **Anchors**: Positions the plugin relative to the `"3D View"` plugin.

### World Statistics Plugin

```xml
<!-- World Statistics -->
<plugin filename="WorldStats" name="World Statistics">
  <gz-gui>
    <title>World Statistics</title>
    <property type="bool" key="showTitleBar">false</property>
    <property type="bool" key="resizable">false</property>
    <property type="double" key="height">110</property>
    <property type="double" key="width">290</property>
    <property type="double" key="z">1</property>
    <property type="string" key="state">floating</property>
    <anchors target="3D View">
      <line own="right" target="right"/>
      <line own="bottom" target="bottom"/>
    </anchors>
  </gz-gui>

  <sim_time>true</sim_time>
  <real_time>true</real_time>
  <real_time_factor>true</real_time_factor>
  <iterations>true</iterations>
  <topic>/world/world_demo/stats</topic>
</plugin>
```

- Displays simulation statistics such as simulation time, real time, real-time factor, and iterations.
- You can customize which statistics to display by setting the corresponding tags to `true`.

### Entity Tree Plugin

```xml
<!-- Entity Tree -->
<plugin filename="EntityTree" name="Entity Tree">
</plugin>
```

- Provides a hierarchical view of all entities in the simulation, including models, lights, and other elements.

**Note**: Remember to close the `<gui>` tag after adding all GUI plugins:

```xml
</gui>
```

## Adding Light to the World

Define a light source using the `<light>` tag:

```xml
<light type="directional" name="sun">
  <cast_shadows>true</cast_shadows>
  <pose>0 0 10 0 0 0</pose>
  <diffuse>0.8 0.8 0.8 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <attenuation>
    <range>1000</range>
    <constant>0.9</constant>
    <linear>0.01</linear>
    <quadratic>0.001</quadratic>
  </attenuation>
  <direction>-0.5 0.1 -0.9</direction>
</light>
```

- **`type="directional"`**: Specifies the light type (options include `point`, `spot`, and `directional`).
- **`<cast_shadows>`**: When set to `true`, the light source casts shadows.
- **`<pose>`**: Sets the light's position (`X Y Z`) and orientation (`Roll Pitch Yaw`) relative to the world.
- **Color Properties**:
  - **`<diffuse>`**: The diffuse color component of the light.
  - **`<specular>`**: The specular color component.
- **Attenuation Properties**:
  - **`<range>`**: The effective range of the light.
  - **`<constant>`**, **`<linear>`**, **`<quadratic>`**: Factors that determine how the light intensity diminishes over distance.
- **`<direction>`**: Specifies the direction the light is pointing (applicable for directional and spotlights).

## Adding Models to the World

Instead of building models from scratch, we can use pre-existing models from **Gazebo Fuel**, which hosts a vast collection of models.

### Including a Model Using URI

To add a model from Gazebo Fuel:

1. Visit the [Gazebo Fuel website](https://app.gazebosim.org/fuel).
2. Choose a model you like.
3. Click the `<>` icon to copy the SDF snippet.
4. Paste the snippet into your world file before the closing `</world>` tag:

   ```xml
   <include>
     <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke</uri>
   </include>
   ```

- **Note**: This method downloads the model at runtime.

### Downloading and Including a Model Locally

If you prefer to have the model available locally:

1. Download the model from Gazebo Fuel and place it in a directory.
2. Set the `GZ_SIM_RESOURCE_PATH` environment variable to point to the directory containing your models:

   ```bash
   export GZ_SIM_RESOURCE_PATH="$HOME/world_path"
   ```

3. Include the model in your world file using the local URI:

   ```xml
   <include>
     <uri>model://Coke</uri>
   </include>
   ```

- **Directory Structure Example**:

  ```
  world_tutorial/
  ├── Coke/
  └── world.sdf
  ```

### Setting Model Position and Spawning Multiple Instances

You can position the model using the `<pose>` tag and assign unique names when spawning multiple instances:

```xml
<include>
  <name>Coke0</name>
  <pose>0 0 0 0 0 0</pose>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke</uri>
</include>
<include>
  <name>Coke1</name>
  <pose>0 0.1 0 0 0 0</pose>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Coke</uri>
</include>
```

- **`<name>`**: Assigns a unique identifier to each model instance.
- **`<pose>`**: Positions the models at different locations to prevent overlap.

## Running the Simulation

To launch your custom world:

```bash
gz sim world.sdf
```

- **Observations**:
  - The models should appear in the simulation according to their specified poses.
  - Use the GUI plugins to interact with and inspect the world elements.
