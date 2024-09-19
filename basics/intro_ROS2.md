
Created By: Khaled Gabr

# Introduction To ROS 2

## **Setting up ROS 2 Development Environment**

### **1. Introduction to Development Environment**

In this course, we will use **Linux Ubuntu 22.04** as the base operating system, and the Robot Operating System (ROS) version used will be **ROS 2 Humble Hawksbill**. It is essential to set up the development environment correctly to ensure all components of ROS 2 work smoothly. This guide will walk you through the installation of Ubuntu 22.04, ROS 2 Humble, and the necessary development tools to get started.

### **2. Installing the Base Operating System: Linux Ubuntu 22.04**

ROS 2 is designed to work best on Linux, particularly Ubuntu. To set up ROS 2 Humble, you need to install Ubuntu 22.04 on your machine. Follow these steps to install Ubuntu:

1. **Download Ubuntu 22.04**:
   - Go to the [Ubuntu download page](https://ubuntu.com/download) and download the 22.04 LTS version.

2. **Create a Bootable USB Drive**:
   - Use a tool like **Rufus** (for Windows) or **Etcher** (for macOS/Linux) to create a bootable USB drive with the downloaded Ubuntu ISO file.

3. **Install Ubuntu 22.04**:
   - Insert the bootable USB drive into your computer and restart it.
   - Follow the on-screen instructions to install Ubuntu 22.04, ensuring you allocate enough disk space for the installation and development work.

### **3. Installing the Robot Operating System: ROS 2 Humble**

With Ubuntu 22.04 installed, we will now install ROS 2 Humble. Follow these steps:

1. **Setup Sources**:

   ```bash
   sudo apt update && sudo apt install -y software-properties-common
   sudo add-apt-repository universe
   ```

2. **Add the ROS 2 Repository**:

   ```bash
   sudo apt update && sudo apt install -y curl gnupg2 lsb-release
   curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
   sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
   ```

3. **Install ROS 2 Humble**:

   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

4. **Setup Environment**:
   After installation, you need to set up your environment:

   ```bash
   source /opt/ros/humble/setup.bash
   ```

5. **Install Dependencies**:
   You will also need to install dependencies for building and running ROS 2 packages:

   ```bash
   sudo apt install -y python3-rosdep python3-rosinstall-generator python3-wstool python3-rosinstall build-essential
   sudo rosdep init
   rosdep update
   ```

### **4. Installing ROS 2 Development Tools**

To develop effectively in ROS 2, you need a set of essential development tools. Run the following commands to install these tools:

1. **Basic Development Tools**:

   ```bash
   sudo apt update && sudo apt install -y \
   build-essential cmake git libbullet-dev python3-colcon-common-extensions python3-flake8 python3-pip \
   python3-pytest-cov python3-rosdep python3-setuptools python3-vcstool wget
   ```

2. **Python Development Tools**:

   ```bash
   python3 -m pip install -U \
   argcomplete flake8-blind-except flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated \
   flake8-docstrings flake8-import-order flake8-quotes pytest-repeat pytest-rerunfailures pytest
   ```

3. **Additional Libraries**:

   ```bash
   sudo apt install --no-install-recommends -y libasio-dev libtinyxml2-dev libcunit1-dev
   ```

### **5. Setting Up Run Commands for Convenience**

To streamline your ROS 2 workflow, it's useful to add frequently used commands, environment variables, and aliases to your `.bashrc` file. This file runs every time you open a new terminal, saving you from retyping commands.

1. **Open the `.bashrc` File**:
   You can use any text editor you're comfortable with:

   ```bash
   nano ~/.bashrc
   ```

   Or use other editors like `vim` or `xed`.

2. **Add the Following Lines to `.bashrc`**:

   ```bash
   source /opt/ros/humble/setup.bash
   source ~/colcon_ws/install/local_setup.bash
   source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
   source /usr/share/vcstool-completion/vcs.bash
   source /usr/share/colcon_cd/function/colcon_cd.sh

   export _colcon_cd_root=~/colcon_ws
   export ROS_DOMAIN_ID=7
   export ROS_NAMESPACE=robot1

   export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
   # Uncomment the desired RMW implementation:
   # export RMW_IMPLEMENTATION=rmw_connext_cpp
   # export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   # export RMW_IMPLEMENTATION=rmw_gurumdds_cpp

   export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity} {time}] [{name}]: {message} ({function_name}() at {file_name}:{line_number})'
   export RCUTILS_COLORIZED_OUTPUT=1
   export RCUTILS_LOGGING_USE_STDOUT=0
   export RCUTILS_LOGGING_BUFFERED_STREAM=1

   alias cw='cd ~/colcon_ws'
   alias cs='cd ~/colcon_ws/src'
   alias ccd='colcon_cd'

   alias cb='cd ~/colcon_ws && colcon build --symlink-install'
   alias cbs='colcon build --symlink-install'
   alias cbp='colcon build --symlink-install --packages-select'
   alias cbu='colcon build --symlink-install --packages-up-to'
   alias ct='colcon test'
   alias ctp='colcon test --packages-select'
   alias ctr='colcon test-result'

   alias rt='ros2 topic list'
   alias re='ros2 topic echo'
   alias rn='ros2 node list'

   alias killgazebo='killall -9 gazebo & killall -9 gzserver & killall -9 gzclient'

   alias af='ament_flake8'
   alias ac='ament_cpplint'

   alias testpub='ros2 run demo_nodes_cpp talker'
   alias testsub='ros2 run demo_nodes_cpp listener'
   alias testpubimg='ros2 run image_tools cam2image'
   alias testsubimg='ros2 run image_tools showimage'
   ```

3. **Save and Apply the Changes**:
   Save the file and run:

   ```bash
   source ~/.bashrc
   ```

   This command reloads the `.bashrc` file to apply the new settings immediately.

---

## **Key Concepts and Features of ROS 2**

### **Why ROS 2?**

ROS 2 (Robot Operating System 2) is an open-source software development kit (SDK) for building robotic applications, designed to cater to both research and commercial use. Released by Open Robotics, ROS 2 builds upon the success and experience of ROS 1, which has been used worldwide in various robotics applications since 2007.

This section explores the main reasons why ROS 2 has become the preferred choice for robotics developers, summarizing the key concepts and unique features that make ROS 2 stand out in the field of robotics.

![alt text](why_ROS2.png)

### **1. Shortened Time to Market**

One of the most significant benefits of using ROS 2 is the reduction in time needed to develop and deploy robotic applications. ROS 2 provides a comprehensive set of tools, libraries, and functions that eliminate the need for developers to create a custom framework from scratch.

This allows developers to focus on the critical aspects of robotic development rather than spending valuable time selecting communication methods, building debugging and visualization tools, or dealing with proprietary software limitations. Moreover, as an open-source platform developed by a community rather than a single commercial entity, ROS 2 offers flexibility in how and where it is used, allowing modifications as needed to fit specific project requirements.

### **2. Designed for Production**

ROS 2 has been developed with a strong emphasis on industrial needs, unlike its predecessor, ROS 1, which was more academic in nature. Over the years, ROS has become the de facto standard for robotics research and development worldwide. ROS 2 takes this a step further by incorporating the industry's requirements for high reliability, safety, and performance.

The design, development, and project management of ROS 2 are based on real-world feedback from industrial stakeholders, ensuring it is ready for both prototyping and full-scale production.

### **3. Multi-Platform Support**

ROS 2 is designed to be flexible and cross-platform, supporting and tested on **Linux, Windows, and macOS**. This allows developers to build and deploy robotic applications across various platforms, including desktop environments, embedded systems, and real-time operating systems (RTOS).

This multi-platform capability enables seamless development and deployment, providing a consistent experience whether you are managing autonomy, backend operations, or user interfaces.

### **4. Multi-Domain Usability**

Much like its predecessor, ROS 2 is designed to be versatile, supporting a wide range of robotic applications across multiple domains:

- **Indoor and Outdoor Robotics**: From warehouse automation to field robots.
- **Home and Automotive Applications**: Enabling smart home devices and autonomous vehicles.
- **Underwater and Space Exploration**: Powering subsea robots and extraterrestrial rovers.
- **Consumer to Industrial**: Used in applications ranging from personal robots to large-scale industrial automation.

ROS 2 offers a consistent development experience across these varied environments, making it a universal tool for all kinds of robotics applications.

### **5. Vendor Flexibility**

ROS 2 provides flexibility in vendor selection by abstracting robotics libraries and applications from the underlying communication functions. This abstraction allows developers to use a range of communication methodologies, from open-source solutions to proprietary ones, depending on their project needs.

Core libraries and user applications can be developed, modified, and extended atop this abstraction, ensuring a tailored fit for different use cases and environments.

### **6. Based on Open Standards**

ROS 2 uses widely adopted industry standards for communication, such as **IDL (Interface Definition Language), DDS (Data Distribution Service), and DDS-I RTPS (Real-Time Publish-Subscribe)**. These standards are commonly used across many sectors, including manufacturing, aerospace, and automotive industries, ensuring compatibility and interoperability across a wide range of applications.

### **7. Open-Source Licensing with Broad Permissions**

The ROS 2 codebase is licensed under the **Apache 2.0 license**, which provides a wide scope of discretionary permission for developers and businesses. This means that you can use ROS 2 freely without affecting your intellectual property rights, whether you are developing for research purposes or commercial products.

### **8. Global Community**

The ROS project has fostered a global community of hundreds of thousands of developers and users over the past decade. This community has contributed to the creation and improvement of a vast ecosystem of software packages and tools. ROS 2 continues this tradition of being developed "by the community, for the community," ensuring that it remains up-to-date, well-supported, and aligned with the needs of its users.

### **9. Strong Industrial Support**

ROS 2 enjoys robust support from the robotics industry. Many companies, both large and small, are contributing resources and expertise to the development of ROS 2. The **ROS 2 Technical Steering Committee** includes members from various companies who provide input and feedback, ensuring that the platform meets industrial standards and remains competitive.

### **10. Interoperability with ROS 1**

For developers who have already invested in ROS 1, ROS 2 provides a bridge for interoperability. This bridge allows for bidirectional communication between ROS 1 and ROS 2 systems, enabling existing ROS 1 applications to be tested with ROS 2. This approach allows for a gradual transition from ROS 1 to ROS 2 based on your needs and available resources.

---

## **Features of ROS 2 through the Differences between ROS 1 and ROS 2**

### **1. ROS 2 Overview**

ROS 2 (Robot Operating System 2), released by Open Robotics on December 8, 2017, is a complete overhaul of the existing ROS (hereinafter referred to as ROS 1). It is not backward compatible with ROS 1, as it was developed to address several limitations and add new functionalities required in the current robotic development landscape.

The development history of ROS 2 reflects a shift towards a more robust, real-time, and flexible platform suitable for both academic research and commercial applications. Below is a timeline of ROS 2 releases:

- **2024.05.23 - ROS 2 Jazzy Jalisco (LTS, 5 years support)**
- **2023.05.23 - ROS 2 Iron Irwini**
- **2022.05.23 - ROS 2 Humble Hawksbill (LTS, 5 years support)**
- **2021.05.23 - ROS 2 Galactic Geochelone**
- **2020.06.05 - ROS 2 Foxy Fitzroy (LTS, 3 years support)**
- **2019.11.22 - ROS 2 Eloquent Elusor**
- **2019.05.31 - ROS 2 Dashing Diademata (First LTS, 2 years support)**
- **2018.12.14 - ROS 2 Crystal Clemmys**
- **2018.07.02 - ROS 2 Bouncy Bolson**
- **2017.12.08 - ROS 2 Ardent Apalone (1st version)**

#### **ROS 2 Pre-releases:**

- **2017.09.13 - ROS 2 Beta3 (code name R2B3)**
- **2017.07.05 - ROS 2 Beta2 (code name R2B2)**
- **2016.12.19 - ROS 2 Beta1 (code name Asphalt)**
- **2016.10.04 - ROS 2 Alpha8 (code name Hook.and.Loop)**
- **2016.07.14 - ROS 2 Alpha7 (code name Glue Gun)**
- **2016.06.02 - ROS 2 Alpha6 (code name Fastener)**
- **2016.04.06 - ROS 2 Alpha5 (code name Epoxy)**
- **2016.02.17 - ROS 2 Alpha4 (code name Duct Tape)**
- **2015.12.18 - ROS 2 Alpha3 (code name Cement)**
- **2015.11.03 - ROS 2 Alpha2 (code name Baling Wire)**
- **2015.08.31 - ROS 2 Alpha1 (code name Anchor)**

### **ROS 1 Limitations and Need for ROS 2**

ROS 1, which began development in 2007, was initially designed for academic and research purposes. Developed by Willow Garage for the personal service robot PR2, ROS 1 has been widely adopted by universities, research institutes, industries, and hobbyists. However, the development environment of ROS 1 inherited several limitations from its origins:

- `Single robot use`
- `Workstation-class computer dependency`
- `Exclusive Linux environment`
- `Lack of real-time control`
- `Requirement for a stable network environment`
- `Primarily for academic research purposes`

These limitations are at odds with the modern requirements of robotics development, which demands:

- `Multi-robot support`
- `Usage in embedded systems`
- `Real-time control`
- `Flexibility in unstable network environments`
- `Multi-platform support (Linux, macOS, Windows)`
- `Integration with the latest technologies (e.g., Zeroconf, Protocol Buffers, ZeroMQ, WebSockets, DDS)`
- `Commercial product support`

To address these needs, ROS 2 was developed as a separate version with next-generation features, allowing existing ROS 1 users to continue using ROS 1 if they do not require the new features. A bridge program, `ros1_bridge`, facilitates communication between ROS 1 and ROS 2, allowing both versions to be used together.

### **2. Differences Between ROS 1 and ROS 2**

The following table summarizes the key differences between ROS 1 and ROS 2:

| **Feature**                    | **ROS 1**                                                    | **ROS 2**                                                                          |
|--------------------------------|--------------------------------------------------------------|-------------------------------------------------------------------------------------|
| **Platforms**                  | Linux, macOS                                                 | Linux, macOS, Windows                                                               |
| **Real-time**                  | External frameworks like OROCOS                              | Real-time nodes with proper RTOS and carefully written user code                    |
| **Security**                   | SROS                                                         | SROS 2, DDS-Security, Robotic Systems Threat Model                                   |
| **Communication**              | XMLRPC + TCPROS                                              | DDS (RTPS)                                                                          |
| **Middleware Interface**       | -                                                            | `rmw`                                                                               |
| **Node Manager (Discovery)**   | ROS Master                                                   | No, uses DDS’s dynamic discovery                                                    |
| **Languages**                  | C++03, Python 2.7                                            | C++14 (C++17), Python 3.5+                                                          |
| **Client Library**             | `roscpp`, `rospy`, `rosjava`, `rosnodejs`, and more          | `rclcpp`, `rclpy`, `rcljava`, `rclobjc`, `rclada`, `rclgo`, `rclnodejs`             |
| **Build System**               | `rosbuild` → `catkin` (CMake)                                | `ament` (CMake), Python setuptools (Full support)                                   |
| **Build Tools**                | `catkin_make`, `catkin_tools`                                | `colcon`                                                                            |
| **Build Options**              | -                                                            | Multiple workspace, No non-isolated build, No devel space                           |
| **Version Control System**     | `rosws` → `wstool`, `rosinstall` (`*.rosinstall`)            | `vcstool` (`*.repos`)                                                               |
| **Life Cycle**                 | -                                                            | Node lifecycle                                                                      |
| **Multiple Nodes**             | One node per process                                         | Multiple nodes per process                                                          |
| **Threading Model**            | Single-threaded or multi-threaded execution                  | Custom executors                                                                    |
| **Messages (Topic, Service, Action)** | `*.msg`, `*.srv`, `*.action`                                       | `*.msg`, `*.srv`, `*.action`, `*.idl`                                               |
| **Command Line Interface**     | `rosrun`, `roslaunch`, `ros topic`                           | `ros2 run`, `ros2 launch`, `ros2 topic`                                             |
| **roslaunch**                  | XML                                                          | Python, XML, YAML                                                                   |
| **Graph API**                  | Remapping at startup only                                    | Remapping at runtime                                                                |
| **Embedded Systems**           | `rosserial`, `mROS`                                          | `micro-ROS`, `XEL Network`, `ros2arduino`, `Renesas`, `DDS-XRCE` (`Micro-XRCE-DDS`), `AWS ARCLM` |

### **3. Key Features of ROS 2**

#### **3.1 Platforms**

ROS 2 supports `Linux`, `Windows`, and `macOS`, making it versatile across all major operating systems. It allows for easy installation using binary files, providing greater accessibility, especially for Windows users.

#### **3.2 Real-time Support**

ROS 2 supports real-time capabilities, provided specific hardware, a real-time operating system (`RTOS`), and suitable communication protocols like DDS's RTPS (Real-time Publish-Subscribe Protocol) are used. This makes ROS 2 suitable for time-sensitive applications.

#### **3.3 Security**

Security was a `low` priority in ROS 1, leading to several vulnerabilities. ROS 2 addresses these issues with DDS-based communication, incorporating DDS-Security standards, and introducing tools like SROS 2 (Secure Robot Operating System 2) for enhanced security.

#### **3.4 Communication**

ROS 2 adopts `DDS` (Data Distribution Service) for communication, providing real-time publish-subscribe capabilities, automatic node detection, and enhanced flexibility and security. This marks a significant departure from ROS 1’s in-house TCPROS.

#### **3.5 Middleware Interface**

ROS 2 offers a middleware interface (`rmw`) that supports multiple DDS vendors, allowing developers to choose middleware based on their needs without worrying about API differences.

#### **3.6 Node Manager (Discovery)**

In ROS 2, the ROS Master is `removed`, and nodes are dynamically discovered using `DDS’s` discovery features, eliminating the single point of failure present in ROS 1.

#### **3.7 Programming Languages**

ROS 2 supports modern programming languages, including `C++14` (or C++17) and `Python 3.5+`, moving away from the older C++03 and Python 2.7 used in ROS 1.

#### **3.8 Build System**

ROS 2 introduces `ament`, a more versatile build system supporting both CMake and Python setuptools, compared to the CMake-only `catkin` in ROS 1.

#### **3.9 Build Tools**

ROS 2 uses `colcon` for building, testing, and managing packages, replacing the older `catkin_make` and `catkin_tools` used in ROS 1.

#### **3.10 Build Options**

ROS 2 introduces new build options such as multiple workspaces, isolated builds, and removal of devel space, providing a more flexible and manageable build environment.

#### **3.11 Version Control System**

ROS 2 consolidates various VCS tools into `vcstool`, streamlining version control across different repositories and version control systems.

#### **3.12 Client Libraries**

ROS 2 offers updated client libraries (`rclcpp`, `rclpy`, etc.) with support for multiple programming languages and modern language standards, enhancing flexibility and usability.

#### **3.13 Node Life Cycle**

ROS 2 provides built-in support for managing the lifecycle of nodes, allowing better control over node states and transitions, which was not natively supported in ROS 1.

#### **3.14 Multiple Nodes per Process**

ROS 2 allows multiple nodes to run in a single process using components, increasing efficiency and reducing overhead compared to ROS 1’s single-node-per-process model.

#### **3.15 Threading Model**

ROS 2 introduces custom executors, allowing more granular control over threading models, which provides better performance and flexibility.

#### **3.16 Messages**

ROS 2 uses the Interface Description Language (IDL) to define messages, adding to the existing `.msg`, `.srv`, and `.action` files, providing enhanced serialization and compatibility across various programming languages.

#### **3.17 Command Line Interface**

The ROS 2 `CLI` maintains familiarity with ROS 1 but introduces improvements in command names and reliability, supported by Canonical.

#### **3.18 roslaunch**

ROS 2 supports launching configurations using `Python`, `XML`, and `YAML`, offering more flexibility and functionality compared to the XML-only launch files in ROS 1.

#### **3.19 Graph API**

ROS 2 enables `dynamic remapping` of nodes and topics at `runtime`, providing more flexibility in managing communication graphs.

#### **3.20 Embedded Systems**

ROS 2 expands support for embedded systems with tools like `micro-ROS`, `ros2arduino`, and compatibility with DDS-XRCE, enhancing its applicability in constrained environments.

---

### ROS Message Communication

In both ROS 1 and ROS 2, message communication is a core concept essential for linking different nodes. A **node** is the smallest executable unit in ROS, functioning as a standalone program. Nodes communicate by exchanging **messages** — data types such as integers, floating-point numbers, booleans, strings, or more complex data structures like arrays. The communication methods in ROS are:

- **Topics**: Publish/subscribe mechanism for asynchronous communication.
- **Services**: Synchronous communication mechanism allowing a request-response model.
- **Actions**: Asynchronous communication that supports preemption (cancellation) and is useful for long-running tasks.
- **Parameters**: Configuration settings that nodes can read or modify during runtime.

### ROS 2 and DDS

ROS 2 introduces DDS (Data Distribution Service) as its core communication middleware, replacing the custom-built communication library (TCPROS) used in ROS 1. DDS is an industry-standard protocol by the Object Management Group (OMG) designed for real-time data distribution. It offers several advantages, including:

- **Real-time Publish-Subscribe (RTPS) Protocol**: Enables real-time communication and is suitable for embedded systems.
- **Quality of Service (QoS)**: Allows fine-grained control over data communication properties like `reliability`, `speed`, and `resource usage`.
- **Security**: Uses DDS-Security to address the security gaps present in ROS 1.

![alt text](image.png)

The introduction of DDS in ROS 2 leads to a major architectural change, as it removes the central ROS Master required in ROS 1, allowing for decentralized, dynamic discovery of nodes. This shift enhances scalability and reliability, especially for commercial and industrial applications.

![alt text](image-1.png)

### Key Features of DDS

1. **Industry Standards**: Managed by OMG and widely used across industries such as automotive, aerospace, and defense.
2. **OS Independence**: Supports multiple operating systems, including Linux, Windows, macOS, Android, and VxWorks.
3. **Language Independence**: Compatible with multiple programming languages, enhancing flexibility.
![alt text](image-2.png)
4. **Transport on UDP/IP**: Leverages UDP multicast for efficient data distribution.
5. **Data Centricity**: Focuses on the data being transmitted, enhancing data management and security.
![alt text](image-3.png)
6. **Dynamic Discovery**: Nodes can discover each other dynamically without predefined IPs or ports.
7. **Scalable Architecture**: Scales from small devices to large, distributed systems.
8. **Interoperability**: Supports communication between different DDS implementations, ensuring compatibility.
![alt text](image-4.png)
9. **Quality of Service (QoS)**: Configurable settings for reliability, durability, deadlines, and more.
10. **Security**: Built-in security features to protect data integrity and confidentiality.

### Usage in ROS 2

**Running a Basic Publisher and Subscriber Node**:

- In ROS 2, nodes use DDS middleware (RMW) to communicate.
- Example commands to run a publisher (`talker`) and subscriber (`listener`) node:

  ```bash
  ros2 run demo_nodes_cpp listener
  ros2 run demo_nodes_cpp talker
  ```

**Changing the RMW Implementation**:  

- By default, ROS 2 uses `rmw_fastrtps_cpp`. You can switch to another RMW by setting the `RMW_IMPLEMENTATION` environment variable:
  
  ```bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ros2 run demo_nodes_cpp listener
  ```

**Testing RMW Interoperability**:

- Different RMW implementations (`rmw_cyclonedds_cpp` and `rmw_fastrtps_cpp`) can interoperate, confirming the flexibility of DDS middleware.

**Changing DDS Domain**:  

- ROS 2 supports setting a domain using `ROS_DOMAIN_ID` to separate communication spaces:
  
  ```sh
  export ROS_DOMAIN_ID=11
  ```

**Testing Quality of Service (QoS)**:

- ROS 2 supports various QoS settings to control communication behavior (e.g., `RELIABLE` vs. `BEST_EFFORT`).

---

<https://cafe.naver.com/openrt/24086>

### Installing Packages and Running Nodes: Turtlesim in ROS 2

Let's explore the practical steps to run the `turtlesim` package in ROS 2 and understand its core functionalities and usage.

#### 1. What You Will Learn

Over the next 1-2 weeks, this course will cover the basic packages, nodes, topics, services, actions, parameters, CLI tools, and rqt tools of ROS 2. We will use the `turtlesim` package as a learning tool, which is ideal for beginners due to its simple but illustrative functionalities.

#### 2. Installing the Turtlesim Package

To begin using `turtlesim`, you need to ensure it's installed. If you haven't already installed it as part of the ROS 2 setup, use the following commands:

```bash
sudo apt update
sudo apt install ros-humble-turtlesim
```

This will install the `turtlesim` package, which is widely used in ROS tutorials to help new users learn the basics of ROS.

#### 3. What is Turtlesim?

The `turtlesim` package is a simple simulator created in the early days of ROS to help users understand the fundamental concepts of ROS, such as nodes, topics, services, and parameters. The package includes several nodes that allow a simulated turtle to move around a graphical environment.

The name "turtlesim" and the use of a turtle icon have historical significance in the ROS community. The turtle became a mascot due to its educational use in the early programming language, Logo, and its turtle graphics concept. The idea was to make programming accessible and fun, using a turtle robot that could be programmed to move and draw.

#### 4. Exploring the Turtlesim Package and Its Nodes

To check the installed packages and nodes in your ROS 2 development environment:

- **List all installed packages**:

```bash
ros2 pkg list
```

- **List nodes available in the `turtlesim` package**:

```bash
ros2 pkg executables turtlesim
```

This will show you the following nodes:

- `draw_square`: Moves the turtle in a square pattern.
- `mimic`: Mirrors movements across multiple turtles.
- `turtle_teleop_key`: Allows controlling the turtle with keyboard input.
- `turtlesim_node`: A 2D simulator that moves the turtle based on received velocity commands.

#### 5. Running Turtlesim Nodes

To visualize and control the turtle, run the following nodes in separate terminal windows:

- **Run the turtlesim simulator**:

```bash
ros2 run turtlesim turtlesim_node
```

- **Run the keyboard teleoperation node**:

```bash
ros2 run turtlesim turtle_teleop_key
```

The `turtlesim_node` will open a window with a turtle on a blue background. You can use the arrow keys to move the turtle using the `turtle_teleop_key` node.

#### 6. Querying Nodes, Topics, Services, and Actions

To explore what nodes are currently running, which topics are available, and what services and actions are in use, use the following commands:

- **List all active nodes**:

```bash
ros2 node list
```

- **List all available topics**:

```bash
ros2 topic list
```

- **List all available services**:

```bash
ros2 service list
```

- **List all available actions**:

```bash
ros2 action list
```

These commands help you understand the communication structure and the various functionalities available in your current ROS 2 environment.

#### 7. Visualizing Nodes and Topics with `rqt_graph`

The `rqt_graph` tool provides a graphical representation of nodes, topics, and actions in the current development environment.

To run `rqt_graph`, use the following command:

```bash
rqt_graph
```

This tool will display nodes as circles, topics or actions as squares, and arrows representing the direction of message flow. Note that services are not displayed in `rqt_graph` because they are only used momentarily when needed.

---

# Data Communication with ROS 2 Nodes

## 1. Node and Message Communication

As explained in the previous lecture, a **node** refers to the smallest unit of executable process, which means a single executable program. In ROS, programs are divided into smaller execution units for modularity.

For example, a node may be a camera driver that outputs the original image, a filter node that processes the image, a node that extracts features, another for object detection, a path planning node, a motor driver node, or one that moves the robot along the path. Dividing responsibilities among nodes reduces dependencies and increases reusability of each node for different tasks.

To link numerous nodes in a ROS system, input and output data must be exchanged. This data is called a **message**, and the method of exchange is **message communication**. Messages can be of various types like integers, floating points, booleans, or strings. More complex data structures, like arrays or messages containing other messages, can also be used. Based on how messages are exchanged, they can be categorized into **topics**, **services**, **actions**, and **parameters**.

In a system with nodes such as Node A, Node B, and Node C (Figure 1), messages are exchanged between them to allow coordination. As the number of tasks increases, more nodes are added, making the system scalable.

![alt text](image-5.png)

### Topic Communication

A **topic** represents a communication link between a **publisher** (Node A) that sends messages and a **subscriber** (Node B, Node C) that receives them. It works asynchronously in a one-way transmission (Figure 2). Topics can support 1:N, N:1, or N:N communication, making it the most common form of message exchange in ROS.

![alt text](image-6.png)

### Service Communication

**Service** communication is synchronous, providing two-way message transmission between a **Service Client** and a **Service Server** (Figure 3). In this model, the client makes a request, and the server processes the request and responds with the result. The requests and responses are variations of `srv` messages, which are derived from `msg` messages.

![alt text](image-7.png)

### Action Communication

An **action** is a combination of asynchronous and synchronous communication, involving three phases: **goal**, **feedback**, and **result**. This is shown in Figure 4, where Node A sets an action goal for Node B. Node B performs the task, providing intermediate feedback and a final result. Actions combine aspects of both topics and services (Figure 5).

- **Goal**: A service-like message exchange to set a task.
- **Feedback**: Topic-like asynchronous updates on progress.
- **Result**: A final service-like message to deliver the task outcome.

![alt text](image-8.png)![alt text](image-9.png)

### Parameters

**Parameters** provide a way to set and retrieve configuration values within nodes (Figure 6). Like services, parameter servers and clients interact to modify or access global and local settings for nodes. These parameters can be changed and retrieved using service-like message communication.

![alt text](image-10.png)

---

## 2. Run Node (ros2 run)

To run a node, use the `ros2 run` command as shown below to run a specific node of a specific package. The two commands below run the `turtlesim_node` and `turtle_teleop_key` nodes of the `turtlesim` package, respectively.

```bash
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

![alt text](image-11.png)

There are two main ways to run a node:

- `ros2 run` to run a single node,
- `ros2 launch` to run one or more nodes.

You can also use tools like `rqt`, `rqt_graph`, and `rviz2` to visualize and control nodes. The following command runs `rqt_graph`, a tool that displays message communication between nodes as a graph.

```bash
rqt_graph
```

![alt text](image-12.png)

## 3. Node List (ros2 node list)

To see the list of nodes currently running in the development environment, you can check using the `ros2 node list` command as shown below:

```bash
ros2 node list
 /rqt_gui_py_node_28168
 /teleop_turtle
 /turtlesim
```

The list displays three nodes, including `rqt_gui_py_node_28168`, `teleop_turtle`, and `turtlesim`. Note that node names can differ from the node file name. For example, `turtlesim_node` is executed with the name `turtlesim`.

You can also run multiple instances of the same node, but with different names. For example, if you want to change the node name:

```bash
ros2 run turtlesim turtlesim_node __node:=new_turtle
```

Running this will add a new node named `new_turtle` to the list. Using `rqt_graph`, you will see the new node along with the others, as shown in Figure 9.

```bash
ros2 node list
 /rqt_gui_py_node_29017
 /teleop_turtle
 /new_turtle
 /turtlesim
```

![alt text](image-13.png)

## 4. Node Information (ros2 node info)

You can also check the details of any node using the `ros2 node info` command. Below is an example for the `turtlesim` node:

```bash
ros2 node info /turtlesim
 /turtlesim
   Subscribers : 
     /parameter_events : rcl_interfaces/msg/ParameterEvent
     /turtle1/cmd_vel : geometry_msgs/msg/Twist
   Publishers : 
     /parameter_events : rcl_interfaces/msg/ParameterEvent
     /rosout : rcl_interfaces/msg/Log
     /turtle1/color_sensor : turtlesim/msg/Color
     /turtle1/pose : turtlesim/msg/Pose
   Service Servers : 
     /clear : std_srvs/srv/Empty
     /kill : turtlesim/srv/Kill
     /reset : std_srvs/srv/Empty
     /spawn : turtlesim/srv/Spawn
     /turtle1/set_pen : turtlesim/srv/SetPen
     /turtle1/teleport_absolute : turtlesim/srv/TeleportAbsolute
     /turtle1/teleport_relative : turtlesim/srv/TeleportRelative
     /turtlesim/describe_parameters : rcl_interfaces/srv/DescribeParameters
     /turtlesim/get_parameter_types : rcl_interfaces/srv/GetParameterTypes
     /turtlesim/get_parameters : rcl_interfaces/srv/GetParameters
     /turtlesim/list_parameters :rcl_interfaces/srv/ListParameters
     /turtlesim/set_parameters : rcl_interfaces/srv/SetParameters
     /turtlesim/set_parameters_atomically : rcl_interfaces/srv/SetParametersAtomically
   Action Servers : 
     /turtle1/rotate_absolute : turtlesim/action/RotateAbsolute
   Action Clients : 
```

Similarly, the `teleop_turtle` node:

```bash
ros2 node info /teleop_turtle 
 /teleop_turtle
   Subscribers : 
     /parameter_events : rcl_interfaces/msg/ParameterEvent
   Publishers : 
     /parameter_events : rcl_interfaces/msg/ParameterEvent
     /rosout : rcl_interfaces/msg/Log
     /turtle1/cmd_vel : geometry_msgs/msg/Twist
   Service Servers : 
     /teleop_turtle/describe_parameters : rcl_interfaces/srv/DescribeParameters
     /teleop_turtle/get_parameter_types : rcl_interfaces/srv/GetParameterTypes
     /teleop_turtle/get_parameters : rcl_interfaces/srv/GetParameters
     /teleop_turtle/list_parameters : rcl_interfaces/srv/ListParameters
     /_turtle/set_parameters : rcl_interfaces/srv/SetParameters
     /teleop_turtle/set_parameters_atomically : rcl_interfaces/srv/SetParametersAtomically
   Action Clients : 
     /turtle1/rotate_absolute : turtlesim/action/RotateAbsolute
```

---

# ROS 2 Topic

## 1. Topic

A **topic** can be seen as a communication method between a **Publisher** (which publishes a message) and a **Subscriber** (which subscribes to a message), as illustrated by the `Node A - Node B` in Figure 1. This communication is asynchronous and typically one-way. Topics can support not only 1:1 communication but also 1:N communication, such as `Node A - Node B` and `Node A - Node C` in Figure 2. N:1 and N:N communications are also possible, depending on the configuration.

A node like `Node A` in Figure 2 can act as both a **Publisher** for multiple topics (e.g., Topic A, Topic C) and as a **Subscriber** for topics (e.g., Topic D). This flexibility is frequently used in ROS, and more than 70% of ROS programming involves topics, making them the most widely used communication method. Topics’ asynchronous nature makes them ideal for tasks where sensor values must be continuously transmitted and information must be exchanged.

![alt text](image-15.png)
![alt text](image-16.png)

## 2. Check the topic list (ros2 topic list)

First, let’s practice using topics with the familiar **turtlesim** package. Run the `turtlesim_node` as shown below, which will display the `turtlesim` environment as seen in Figure 3:

```bash
ros2 run turtlesim turtlesim_node
```

![alt text](image-17.png)

Next, we will check the topic information of the turtlesim_node (`/turtlesim`) using the `ros2 node info` command. The command shows that the turtlesim node subscribes to `/turtle1/cmd_vel` (a message of type `geometry_msgs/msg/Twist`), and publishes the `color_sensor` and `pose` messages.

```bash
ros2 node info /turtlesim
 /turtlesim
   Subscribers : 
     /turtle1/cmd_vel : geometry_msgs/msg/Twist 
   Publishers : 
     /turtle1/color_sensor : turtlesim/msg/Color
     /turtle1/pose : turtlesim/msg/Pose 
   Services : 
     /clear : std_srvs/srv/Empty
     /kill : turtlesim/srv/Kill
```

To check all running topics, you can use the `ros2 topic list -t` command. This lists the topics along with their message types.

```bash
ros2 topic list -t
 /parameter_events [rcl_interfaces/msg/ParameterEvent] 
 /rosout [rcl_interfaces/msg/Log] 
 /turtle1/cmd_vel [geometry_msgs/msg/Twist] 
 /turtle1/color_sensor [turtlesim/msg/Color] 
 /turtle1/pose [turtlesim/msg/Pose]
```

Currently, no topics are being actively exchanged since only the turtlesim node is running. You can visualize this with `rqt_graph` as shown in Figure 4:

```bash
rqt_graph
```

![alt text](image-18.png)

Now, let’s run the `turtle_teleop_key` node. Refreshing the `rqt_graph` using the `Refresh ROS graph` button (Figure 5) will show that turtlesim subscribes to the `/turtle1/cmd_vel` topic published by the `teleop_turtle` node (Figure 6):

```bash
ros2 run turtlesim turtle_teleop_key
```

![alt text](image-19.png)![alt text](image-20.png)

If the `/turtle1/color_sensor` and `/turtle1/pose` topics are not visible, it’s because no nodes are subscribing to them yet. Unchecking `Dead sinks` and `Leaf topics` in `rqt_graph` will reveal all topics, as shown in Figure 7:

![alt text](image-21.png)

## 3. Check topic information (ros2 topic info)

You can also check detailed information about a topic using the `ros2 topic info` command. For instance, to get information about the `/turtle1/cmd_vel` topic:

```bash
ros2 topic info /turtle1/cmd_vel
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscriber count: 1
```

## 4. Check topic contents (ros2 topic echo)

The `ros2 topic echo` command displays the contents of a specific topic in real time. For instance, to view the current velocity of the turtle as you move it using the arrow keys in the `teleop_turtle` terminal, run:

```bash
ros2 topic echo /turtle1/cmd_vel
```

The output will show values such as:

```bash
linear:
  x: 1.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
```

## 5. Check topic bandwidth (ros2 topic bw)

To check the bandwidth usage of a topic, use the `ros2 topic bw` command. The example below shows the bandwidth for the `/turtle1/cmd_vel` topic as 1.74 KB/s:

```bash
ros2 topic bw /turtle1/cmd_vel
average: 1.74KB/s
```

## 6. Check topic cycle (ros2 topic hz)

To check how frequently a topic is published, use the `ros2 topic hz` command. The example below shows that the `/turtle1/cmd_vel` topic is published at an average rate of 33.2 Hz:

```bash
ros2 topic hz /turtle1/cmd_vel
average rate: 33.212
```

## 7. Check topic delay time (ros2 topic delay)

To measure the delay time of a topic that includes a header stamp message, use the `ros2 topic delay` command:

```bash
ros2 topic delay /turtle1/cmd_vel
```

## 8. Publish topic (ros2 topic pub)

You can publish a topic using the `ros2 topic pub` command. For example, to set the turtle’s linear and angular velocities to 2.0 m/s and 1.8 rad/s respectively, run:

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

This command will move the turtle as shown in Figure 8:

![alt text](image-22.png)
If you want to publish continuously, you can use the `--rate 1` option to publish at a frequency of 1 Hz:

```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

This results in continuous movement, as in Figure 9:
![alt text](ezgif-6-78fccd7b23e1.gif)

By adjusting the angular speed, you can create more complex patterns, as seen in Figure 10:

![alt text](image-24.png)

## 9. Record topics (ros2 bag record)

ROS allows you to record topics to a file using the `ros2 bag record` command. For example, to record the `/turtle1/cmd_vel` topic:

```bash
ros2 bag record /turtle1/cmd_vel
```

## 10. View bag information (ros2 bag info)

To view details about a recorded rosbag file, use the `ros2 bag info` command:

```bash
ros2 bag info rosbag2_2020_09_04-08_31_06/
```

---

## 11. Bag Playback (ros2 bag play)

Once the rosbag file has been recorded and you’ve reviewed its contents, you can replay the recorded topics using `ros2 bag play`. First, stop the `turtlesim_node`, restart it, initialize it, and then play the rosbag. The topics will replay according to the original recording time. You can observe the replay using `ros2 topic echo /turtle1/cmd_vel` or by watching the movement of the turtle in `turtlesim`, as shown in Figure 11:

```bash
ros2 bag play rosbag2_2020_09_04-08_31_06/
[INFO]: Opened database 'rosbag2_2020_09_04-08_31_06/'
```

![alt text](image-25.png)

---

## 12. ROS Interface

In ROS, **topics**, **services**, and **actions** are used to exchange data between nodes. The format of the data used is called the **ROS interface**. ROS interfaces consist of:

- **msg**: used for topics.
- **srv**: used for services.
- **action**: used for actions.

These interfaces can use basic data types like integers, floats, and booleans, as well as more complex structures, including arrays or nested messages.

---

## 13. Message Interface (msg)

The `/turtle1/cmd_vel` topic we’ve been discussing uses the `geometry_msgs/msg/Twist` message type, which belongs to the `geometry_msgs` package. The `Twist` message contains two parts:

- **Vector3 linear**
- **Vector3 angular**

Each part contains three `float64` values: `x`, `y`, and `z`. This allows for the representation of 3 translational velocities and 3 rotational velocities, as shown in Figure 12.

![alt text](image-26.png)

To inspect the structure of these messages, you can use the `ros2 interface show` command:

```bash
ros2 interface show geometry_msgs/msg/Twist
Vector3 linear
Vector3 angular
```

```bash
ros2 interface show geometry_msgs/msg/Vector3
float64 x
float64 y
float64 z
```

---

### Other ROS Interface Commands

- **list**: Shows all the msg, srv, and action messages available in the environment.
- **packages**: Lists all the packages containing interfaces.
- **package [package_name]**: Displays interfaces of the specified package.
- **proto [interface]**: Shows the default type of the interface.

```bash
ros2 interface list
Messages:
    action_msgs/msg/GoalInfo
    action_msgs/msg/GoalStatus
    action_msgs/msg/GoalStatusArray
    (omitted)

Services:
    action_msgs/srv/CancelGoal
    composition_interfaces/srv/ListNodes
    (omitted)

Actions:
    action_tutorials_interfaces/action/Fibonacci
    example_interfaces/action/Fibonacci
    (omitted)
```

```bash
ros2 interface packages
action_msgs
action_tutorials_interfaces
actionlib_msgs
builtin_interfaces
(omitted)
```

```bash
ros2 interface package turtlesim
turtlesim/srv/TeleportAbsolute
turtlesim/srv/SetPen
turtlesim/msg/Color
turtlesim/action/RotateAbsolute
turtlesim/msg/Pose
turtlesim/srv/Spawn
turtlesim/srv/TeleportRelative
turtlesim/srv/Kill
```

```bash
ros2 interface proto geometry_msgs/msg/Twist
"linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

The `msg` interfaces are used for topics. Additionally, you’ll encounter **srv** and **action** interfaces, which will be explained in detail in the services and actions lectures.

---
