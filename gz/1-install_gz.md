# Gazebo Sim

## Binary Installation on Ubuntu

Harmonic binaries are provided for **Ubuntu Jammy (22.04)** and **Ubuntu Noble (24.04)**. The Harmonic binaries are hosted in the `packages.osrfoundation.org` repository. To install all of them, the metapackage `gz-harmonic` can be installed.

> **WARNING:** For Gazebo Classic users (e.g., `gazebo11`), `gz-harmonic` cannot be installed alongside `gazebo11` by default. To facilitate migration, this can be done using the instructions detailed in [Installing Gazebo11 side by side with new Gazebo](#).

### Installing Necessary Tools

First, install some necessary tools:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

### Installing Gazebo Harmonic

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

All libraries should now be ready to use, and the gz sim app will be ready for execution.

### Uninstalling Binary Installation
If you need to uninstall Gazebo or switch to a source-based installation after installing from binaries, run the following command:

```bash
sudo apt remove gz-harmonic && sudo apt autoremove
```