# CENG-407-408-2024-2025-Sentinel-Autonomous-Discovery-Vehicle

## Sentinel: Autonomous Discovery Vehicle

### Project Description

<div align="justify">
The Sentinel is designed to autonomously explore and navigate unknown environments with precision. As a compact and highly functional vehicle, its main feature is generating a 3D map of the areas scanned by sensors and cameras. These maps enable the Sentinel to effortlessly identify optimal paths and locate key areas. Additionally, it utilizes object detection algorithms to distinguish between static and dynamic objects, ensuring accurate identification and marking of obstacles or objects on the map. This combination of autonomous mapping and object recognition makes the Sentinel an invaluable tool for exploration in challenging environments, such as war zones or dangerous areas contaminated by chemical or nuclear particles.
</div>

### Team Members

| Team Members                                             | Student Numbers |
| -------------------------------------------------------- | :-------------: |
| [Burak ATEŞ ](https://github.com/AtesBurak1)             |    202011010    |
| [Turgut Utku ALTINKAYA](https://github.com/UtkuAltnkaya) |    202011036    |
| [Yunus Emre DİNÇEL](https://github.com/yunusemredincell) |    202011078    |
| [İlteriş SAMUR](https://github.com/ilterissamur)         |    202111056    |
| [Bayram Alper KILIÇ](https://github.com/alperrkilic)     |    202111401    |

### Advisor

- Assist. Prof. Dr. Serdar ARSLAN

### Project Website

- https://sentinelvehicle.wordpress.com/

### Installation

#### Computer

- <a href="https://docs.ros.org/en/jazzy/Installation.html">Install the Robot Operating System.</a>

- Source ROS dependencies.

  ```bash
  source /opt/ros/jazzy/setup.bash
  ```

- Clone the repository.

  ```bash
  git clone https://github.com/CankayaUniversity/ceng-407-408-2024-2025-Sentinel-Autonomous-Discovery-Vehicle.git Sentinel

  cd Sentinel
  ```

- Change to the directory to install computer packages.

  ```bash
  cd sentinel.computer
  ```

- Install workspace-specific ROS dependencies.

  ```bash
  rosdep update && rosdep install --from-paths src --ignore-src -r -y
  ```

- Create a Python virtual environment and install dependencies (necessary for installing pip-only dependencies).

  ```bash
  python3 -m venv .venv --system-site-packages --symlinks

  touch ./.venv/COLCON_IGNORE # Make sure that colcon does not try to build the venv

  source .venv/bin/activate

  pip3 install -r requirements.txt
  ```

- Build ROS packages.

  ```bash
  colcon build
  ```

- Run.

  ```bash
  source install/setup.bash

  ros2 launch sentinel sim.launch.py
  ```

#### Vehicle

#### Frontend

<i><a href="">See Frontend Installation</a></i>
