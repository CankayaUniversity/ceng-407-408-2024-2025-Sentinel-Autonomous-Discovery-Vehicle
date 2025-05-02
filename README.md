### Frontend Installation

- Download NodeJS & Npm.

  ```bash
    sudo apt update
    sudo apt install nodejs npm -y
  ```

- Install rosbridge to connect ROS from the Web.

  ```bash
    sudo apt install ros-jazzy-rosbridge-suite
  ```

- Clone the repository.

  ```bash
  git clone https://github.com/CankayaUniversity/ceng-407-408-2024-2025-Sentinel-Autonomous-Discovery-Vehicle.git -b sentinel-frontend Sentinel-Frontend
  ```

- Change to the directory to install npm packages.

  ```bash
  cd Sentinel-Frontend
  ```

- Install packages (use force to resolve conflicting peer dependencies).

  ```bash
  npm install --force
  ```
- Make sure that you are on the same LAN with the ROS server.

- Start the Application.

  ```bash
  npm run dev
  ```

<p style="text-align: right;">Update Date 02.05.2025</p>
