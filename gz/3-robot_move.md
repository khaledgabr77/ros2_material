# **1. Differential Drive Plugin Addition**

**Plugin Definition:**

```xml
<plugin
    filename="gz-sim-diff-drive-system"
    name="gz::sim::systems::DiffDrive">
    <left_joint>left_wheel_joint</left_joint>
    <right_joint>right_wheel_joint</right_joint>
    <wheel_separation>1.2</wheel_separation>
    <wheel_radius>0.4</wheel_radius>
    <odom_publish_frequency>1</odom_publish_frequency>
    <topic>cmd_vel</topic>
</plugin>
```

**Explanation:**

- **Purpose**: This plugin enables the robot to be controlled via differential drive, which is common for two-wheeled robots. It processes velocity commands and applies appropriate wheel velocities.

- **Parameters**:

  - **`<left_joint>` and `<right_joint>`**: Specify the joints controlling the left and right wheels.
  - **`<wheel_separation>`**: Distance between the centers of the left and right wheels (1.2 meters).
  - **`<wheel_radius>`**: Radius of the wheels (0.4 meters).
  - **`<odom_publish_frequency>`**: Frequency (in Hz) at which odometry data is published.
  - **`<topic>`**: The topic (`cmd_vel`) on which the plugin listens for velocity commands (linear and angular velocities).

- **Functionality**:

  - **Command Reception**: Listens to the `/cmd_vel` topic for `Twist` messages containing `linear.x` and `angular.z` velocities.
  - **Wheel Velocity Calculation**: Converts the desired linear and angular velocities into individual wheel velocities using differential drive kinematics.
  - **Joint Control**: Applies calculated velocities to the specified wheel joints.

---

# **2. Triggered Publisher Plugins for Keyboard Control**

**Plugins Added:**

```xml
<!-- Moving Left -->
<plugin filename="gz-sim-triggered-publisher-system"
        name="gz::sim::systems::TriggeredPublisher">
    <!-- Configuration -->
</plugin>

<!-- Similar plugins for Moving Forward, Right, and Backward -->
```

**Explanation:**

- **Purpose**: These plugins map keyboard keypress events to velocity commands, allowing you to control the robot using keyboard inputs.

- **Key Components**:

  - **`<input>`**:

    - **`type="gz.msgs.Int32"`**: The message type for the input, representing keypress events.
    - **`topic="/keyboard/keypress"`**: The topic on which keypress events are published.
    - **`<match field="data">...</match>`**: Specifies the key code that triggers the plugin. Each arrow key has a unique code.

  - **`<output>`**:

    - **`type="gz.msgs.Twist"`**: The message type for the output, representing velocity commands.
    - **`topic="/cmd_vel"`**: The topic to which the velocity command is published.
    - **Velocity Specification**:

      - **`linear`**: Sets the linear velocity in the x-direction.
      - **`angular`**: Sets the angular velocity around the z-axis.

- **Key Codes and Actions**:

  - **Left Arrow (`16777234`)**:

    - **Action**: Rotate left (turn counter-clockwise).
    - **Velocity**: `linear.x = 0.0`, `angular.z = 0.5`.

  - **Up Arrow (`16777235`)**:

    - **Action**: Move forward.
    - **Velocity**: `linear.x = 0.5`, `angular.z = 0.0`.

  - **Right Arrow (`16777236`)**:

    - **Action**: Rotate right (turn clockwise).
    - **Velocity**: `linear.x = 0.0`, `angular.z = -0.5`.

  - **Down Arrow (`16777237`)**:

    - **Action**: Move backward.
    - **Velocity**: `linear.x = -0.5`, `angular.z = 0.0`.

- **Functionality**:

  - When a keypress event matching the specified key code is detected, the plugin publishes the corresponding `Twist` message to `/cmd_vel`.
  - The differential drive plugin receives the `Twist` message and adjusts the wheel velocities accordingly.

---

# **3. Topics and Messages**

**Now our model is ready. We just need to send commands (messages) to it. These messages will be published (sent) on the `cmd_vel` topic defined above.**

- **Topics**: In Gazebo and ROS, a topic is a named bus over which nodes exchange messages. Our model subscribes to the `cmd_vel` topic to receive velocity commands.

- **Messages**: The messages sent on the `cmd_vel` topic are of type `Twist`, which includes linear and angular velocities.

**Launching the Robot World:**

To start the simulation with your robot:

```bash
gz sim robot_move.sdf
```

**Sending Commands to the Robot:**

In another terminal, you can manually send a velocity command using:

```bash
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.5}, angular: {z: 0.05}"
```

- **Explanation**:

  - **`gz topic`**: Gazebo command-line tool for publishing and subscribing to topics.

  - **`-t "/cmd_vel"`**: Specifies the topic to publish to.

  - **`-m gz.msgs.Twist`**: Specifies the message type.

  - **`-p "linear: {x: 0.5}, angular: {z: 0.05}"`**: Provides the message content, setting a linear velocity of `0.5 m/s` and an angular velocity of `0.05 rad/s`.

- **Note**: Make sure the simulation is running (press the play button if necessary) to see the robot move.

**Understanding the Command:**

- The command sends a `Twist` message to the `/cmd_vel` topic, which the differential drive plugin uses to control the robot's movement.

**Exploring Topics and Messages:**

- You can get help on the `gz topic` command using:

  ```bash
  gz topic -h
  ```

---

## **Moving the Robot Using the Keyboard**

Instead of sending messages manually, you can control the robot using your keyboard. This involves adding two new plugins:

1. **KeyPublisher**
2. **TriggeredPublisher**

### **KeyPublisher Plugin**

- **Purpose**: Captures keyboard key presses and publishes them to a topic.

- **Usage**:

  - **Starting the Simulation**:

    ```bash
    gz sim robot_move.sdf
    ```

  - **Enabling KeyPublisher Plugin**:

    - In the Gazebo GUI, click on the plugins dropdown list (usually represented by vertical ellipsis `⋮` in the top-right corner).

    - Select **Key Publisher** from the list.

  - **Viewing Key Presses**:

    - In another terminal, run:

      ```bash
      gz topic -e -t /keyboard/keypress
      ```

    - Press different keys in the Gazebo window, and you should see key codes displayed in the terminal.

### **Mapping Key Presses to Robot Movement**

- The **TriggeredPublisher** plugin is used to map key presses to `Twist` messages sent to the `/cmd_vel` topic.

- **Functionality**:

  - When a key press message matching a specific key code is detected on the `/keyboard/keypress` topic, the TriggeredPublisher publishes a predefined `Twist` message to `/cmd_vel`.

  - This setup allows you to control the robot's movement in the simulation using your keyboard.

---
