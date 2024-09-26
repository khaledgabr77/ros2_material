### **Overview of the SDF File Structure**

The file is structured as follows:

- **XML Declaration**: Specifies the XML version.
- **SDF Version**: Indicates the SDF format version (`1.8` in this case).
- **World Definition**: Defines the simulation world named `"car_world"`.
  - **Physics Parameters**: Configures the physics engine settings.
  - **Plugins**: Adds functionality through various plugins.
  - **Lighting**: Sets up the lighting in the simulation.
  - **Ground Plane**: Creates a flat ground surface.
  - **Models**: Defines the objects within the world, including the vehicle.

---

### **1. World Definition**

```xml
<world name="car_world">
    <!-- Content -->
</world>
```

The `<world>` tag defines the simulation environment named `"car_world"`. Everything within this tag pertains to the configuration of the world.

---

### **2. Physics Parameters**

```xml
<physics name="1ms" type="ignored">
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
</physics>
```

- **Physics Engine Configuration**:
  - **`<physics>`**: Sets up the physics engine parameters.
    - **`name="1ms"`**: An arbitrary name for this physics profile.
    - **`type="ignored"`**: Specifies the physics engine type (e.g., ODE, Bullet). Here, it's set to `"ignored"`, which may default to the simulator's default engine.
  - **`<max_step_size>`**: Defines the time step size for each simulation iteration (`0.001` seconds).
  - **`<real_time_factor>`**: Controls the speed of the simulation relative to real-time (`1.0` means real-time).

---

### **3. Plugins**

```xml
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
<plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
<plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
```

- **Plugins** enhance the simulation with additional functionalities:
  - **Physics System Plugin**: Handles physics calculations.
  - **User Commands Plugin**: Allows user interactions and commands.
  - **Scene Broadcaster Plugin**: Broadcasts the simulation scene for visualization.

---

### **4. Lighting Configuration**

```xml
<light type="directional" name="sun">
    <!-- Light Properties -->
</light>
```

- **`<light>`**: Defines a directional light named `"sun"`.
  - **`<cast_shadows>true</cast_shadows>`**: Enables shadow casting.
  - **`<pose>0 0 10 0 0 0</pose>`**: Positions the light at `(0, 0, 10)` with no rotation.
  - **Color Properties**:
    - **`<diffuse>0.8 0.8 0.8 1</diffuse>`**: Sets the diffuse color (RGBA).
    - **`<specular>0.2 0.2 0.2 1</specular>`**: Sets the specular color (shininess).
  - **Attenuation**:
    - **`<range>1000</range>`**: The effective range of the light.
    - **`<constant>0.9</constant>`**, **`<linear>0.01</linear>`**, **`<quadratic>0.001</quadratic>`**: Attenuation factors.
  - **`<direction>-0.5 0.1 -0.9</direction>`**: Direction in which the light is pointing.

---

### **5. Ground Plane**

```xml
<model name="ground_plane">
    <!-- Ground Plane Properties -->
</model>
```

- **`<model>`**: Defines a static model named `"ground_plane"`.
  - **`<static>true</static>`**: Indicates that the ground plane doesn't move.
  - **`<link name="link">`**: Contains the visual and collision properties.
    - **Collision and Visual Geometry**:
      - **`<plane>`**: Defines a plane geometry.
        - **`<normal>0 0 1</normal>`**: The normal vector of the plane (pointing up).
        - **`<size>100 100</size>`** (in visual only): The size of the plane.
    - **Material Properties** (in visual only):
      - **`<ambient>0.8 0.8 0.8 1</ambient>`**: Ambient color.
      - **`<diffuse>0.8 0.8 0.8 1</diffuse>`**: Diffuse color.
      - **`<specular>0.8 0.8 0.8 1</specular>`**: Specular color.

---

### **6. Vehicle Model: "vehicle_blue"**

```xml
<model name="vehicle_blue" canonical_link="chassis">
    <!-- Vehicle Properties -->
</model>
```

- **`<model>`**: Defines the vehicle model named `"vehicle_blue"`.
  - **`canonical_link="chassis"`**: Sets the `"chassis"` link as the reference frame for the model.
  - **`<pose relative_to="world">0 0 0 0 0 0</pose>`**: Positions the model at the origin with no rotation. The pose is relative to the `"world"` frame.

---

#### **6.1 Chassis Link**

```xml
<link name="chassis">
    <!-- Chassis Properties -->
</link>
```

- **`<pose relative_to="__model__">0.5 0 0.4 0 0 0</pose>`**: Positions the chassis relative to the model's reference frame (`__model__`), offset by `0.5` meters in the x-direction and `0.4` meters in the z-direction.
- **Inertial Properties**:
  - **`<mass>1.14395</mass>`**: Mass of the chassis.
  - **`<inertia>`**: Inertia matrix components, defining how the mass is distributed.
- **Visual and Collision Geometry**:
  - **`<box>`**: Defines a box shape.
    - **`<size>2.0 1.0 0.5</size>`**: Dimensions of the box (length, width, height).
  - **Material Properties**:
    - **Color**: Set to blue using ambient, diffuse, and specular properties (`0.0 0.0 1.0 1` for RGBA).

---

#### **6.2 Left Wheel Link**

```xml
<link name="left_wheel">
    <!-- Left Wheel Properties -->
</link>
```

- **`<pose relative_to="chassis">-0.5 0.6 0 -1.5707 0 0</pose>`**: Positions the left wheel relative to the chassis.
  - **Position**: `-0.5` meters behind the chassis in x, `0.6` meters to the left in y.
  - **Rotation**: Rotated `-1.5707` radians (~90 degrees) around the x-axis to orient the wheel correctly.
- **Inertial Properties**:
  - **`<mass>1</mass>`**: Mass of the wheel.
  - **Inertia Matrix**: Defines the wheel's resistance to angular acceleration.
- **Visual and Collision Geometry**:
  - **`<cylinder>`**: Defines a cylindrical shape for the wheel.
    - **`<radius>0.4</radius>`**: Radius of the wheel.
    - **`<length>0.2</length>`**: Width of the wheel.
  - **Material Properties**:
    - **Color**: Set to red (`1.0 0.0 0.0 1`).

---

#### **6.3 Right Wheel Link**

- **Similar to the left wheel**, but positioned on the opposite side:
  - **`<pose relative_to="chassis">-0.5 -0.6 0 -1.5707 0 0</pose>`**: The y-position is `-0.6` meters, placing it to the right.

---

#### **6.4 Arbitrary Frame: "caster_frame"**

```xml
<frame name="caster_frame" attached_to="chassis">
    <pose>0.8 0 -0.2 0 0 0</pose>
</frame>
```

- **`<frame>`**: Defines a coordinate frame named `"caster_frame"`, attached to the `"chassis"`.
  - **Pose**: Positioned `0.8` meters ahead in x and `-0.2` meters in z (below the chassis), with no rotation.

---

#### **6.5 Caster Wheel Link**

```xml
<link name="caster">
    <!-- Caster Wheel Properties -->
</link>
```

- **`<pose relative_to="caster_frame"/>`**: Inherits the pose from `"caster_frame"`.
- **Inertial Properties**:
  - **`<mass>1</mass>`**: Mass of the caster wheel.
  - **Inertia Matrix**: Symmetric, with all diagonal elements set to `0.016`.
- **Visual and Collision Geometry**:
  - **`<sphere>`**: Defines a spherical shape for the caster wheel.
    - **`<radius>0.2</radius>`**: Radius of the sphere.
  - **Material Properties**:
    - **Color**: Set to green (`0.0 1 0.0 1`).

---

#### **6.6 Joints**

##### **Left Wheel Joint**

```xml
<joint name="left_wheel_joint" type="revolute">
    <!-- Left Wheel Joint Properties -->
</joint>
```

- **Type**: `"revolute"` joint, allowing rotation around a single axis.
- **Pose**: Positioned relative to the `"left_wheel"` link.
- **Parent and Child**:
  - **`<parent>chassis</parent>`**: The chassis is the fixed part.
  - **`<child>left_wheel</child>`**: The wheel rotates relative to the chassis.
- **Axis of Rotation**:
  - **`<xyz expressed_in="__model__">0 1 0</xyz>`**: The axis is along the y-axis in the model frame.
- **Limits**:
  - **`<lower>`** and **`<upper>`**: Set to negative and positive infinity, allowing unlimited rotation.

##### **Right Wheel Joint**

- **Identical to the left wheel joint**, connecting the `"right_wheel"` to the `"chassis"`.

##### **Caster Wheel Joint**

```xml
<joint name="caster_wheel" type="ball">
    <!-- Caster Wheel Joint Properties -->
</joint>
```

- **Type**: `"ball"` joint, allowing rotation around all three axes (3 degrees of freedom).
- **Parent and Child**:
  - **`<parent>chassis</parent>`**: Connected to the chassis.
  - **`<child>caster</child>`**: The caster wheel can swivel freely.


### **6.6.1 Joint Type Clarification**

**Comment in SDF:**

```xml
<joint name='left_wheel_joint' type='revolute'> <!--continous joint is not supported yet-->
    <!-- Joint configuration -->
</joint>
```

**Explanation:**

- **Revolute vs. Continuous Joints**:

  - **Revolute Joint**: Allows rotation around a single axis within specified limits.
  - **Continuous Joint**: Allows unlimited rotation around a single axis (like a wheel or propeller).

- **Current Choice**: A `revolute` joint is used with limits set to negative and positive infinity to simulate continuous rotation.

- **Reason**: If the simulator or SDF version doesn't support `continuous` joints, using a `revolute` joint with infinite limits is a practical workaround.

---

### **Additional Notes**

- **Angles in Poses**: Specified in radians.
  - For example, `-1.5707` radians is approximately `-90` degrees, which orients the wheels correctly.
- **Reference Frames**:
  - **`relative_to`**: Specifies the frame of reference for positions and orientations.
    - `"__model__"`: The model's reference frame.
    - `"world"`: The global reference frame.
- **Inertia Matrices**:
  - Define how mass is distributed within the link, affecting how it responds to forces and torques. [important!](https://amesweb.info/inertia/mass-moment-of-inertia-calculator.aspx)
- **Materials and Colors**:
  - Defined using RGBA values, where each component ranges from `0` to `1`.
- **relative_to**:
  - Use `relative_to` to specify where an entity is located with respect to another frame at the time of definition.
- **attached_to**:
  - Use `attached_to` to create a new frame that is physically connected to another frame, inheriting all its movements.

---
