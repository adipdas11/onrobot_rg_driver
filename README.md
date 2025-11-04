# OnRobot RG Driver for ROS 2 (Humble and Newer)

A lightweight ROS 2 package for controlling OnRobot **RG2** and **RG6** grippers using **Modbus TCP**.  
This driver exposes simple **ROS 2 services** and a **status publisher**, allowing full control and feedback without custom firmware or vendor SDKs.

---

## 🦾 Features

- Supports both **RG2** and **RG6** grippers  
- Communicates directly via **Modbus TCP (port 502)**  
- Provides clean **service interfaces** for open, close, move, stop, force, and width  
- Publishes live **status feedback** (width, movement, safety flags, etc.)  
- Pure Python implementation using [`pymodbus`](https://pymodbus.readthedocs.io/en/latest/)  
- ROS 2 Humble+ compatible (uses `rclpy` and `rosidl_default_generators`)  

---

## ⚙️ Dependencies

| Dependency | Version / Notes | Purpose |
|-------------|-----------------|----------|
| ROS 2 Humble or newer | – | Core middleware |
| `pymodbus` | **2.5.3** (recommended) | Modbus TCP communication |
| `rclpy`, `ament_cmake`, `ament_cmake_python` | – | ROS 2 node and build system |
| `builtin_interfaces` | – | Time field for `RGStatus.msg` |

### Install `pymodbus`

You must install the correct `pymodbus` version in your Python environment before running the driver:

```bash
pip install pymodbus==2.5.3
If you are using a ROS 2 workspace virtual environment, make sure it uses the same Python interpreter as your ROS 2 installation.

📁 Package Structure
pgsql
Copy code
onrobot_rg_driver/
├── launch/
│   └── rg.launch.py
├── msg/
│   └── RGStatus.msg
├── srv/
│   ├── Open.srv
│   ├── Close.srv
│   ├── MoveToWidth.srv
│   ├── Stop.srv
│   ├── SetForce.srv
│   ├── SetWidth.srv
│   └── GetStatus.srv
├── onrobot_rg_driver/
│   ├── driver_node.py      
│   └── onrobot.py        
├── resource/
│   └── onrobot_rg_driver
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 🚀 Build and Run
### Clone into your workspace:

```bash
cd ~/workspaces/onrobot_ws/src
git clone https://github.com/<your-username>/onrobot_rg_driver.git
```
### Build the package:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
### Launch the node:

```bash
ros2 launch onrobot_rg_driver rg.launch.py
```

## 🧭 Runtime Interfaces
Published Topic
Topic	Type	Description
/status	onrobot_rg_driver/msg/RGStatus	Real-time gripper state (widths, movement, safety flags, etc.)

Example:

```bash
ros2 topic echo /status
```

## 🧩 Services

| **Service** | **Type** | **Purpose** |
|--------------|-----------|-------------|
| `/open` | `onrobot_rg_driver/srv/Open` | Fully open the gripper |
| `/close` | `onrobot_rg_driver/srv/Close` | Fully close the gripper |
| `/move_to_width` | `onrobot_rg_driver/srv/MoveToWidth` | Move to a specific width (in millimeters) |
| `/stop` | `onrobot_rg_driver/srv/Stop` | Immediately stop all motion |
| `/set_force` | `onrobot_rg_driver/srv/SetForce` | Set the target gripping force (in 1/10 Newtons) |
| `/set_width` | `onrobot_rg_driver/srv/SetWidth` | Set the target width register directly |
| `/get_status` | `onrobot_rg_driver/srv/GetStatus` | Retrieve a single `RGStatus` message snapshot |


## Example usage:

```bash
# Open gripper
ros2 service call /open onrobot_rg_driver/srv/Open "{force_n_10th: 400}"

# Move to 50 mm
ros2 service call /move_to_width onrobot_rg_driver/srv/MoveToWidth "{width_mm: 50.0, force_n_10th: 400}"

# Close with 600 (= 60 N)
ros2 service call /close onrobot_rg_driver/srv/Close "{force_n_10th: 600}"

# Get one-shot status
ros2 service call /get_status onrobot_rg_driver/srv/GetStatus "{}"
```

## 🧩 Message Definition

**File:** `onrobot_rg_driver/msg/RGStatus.msg`

```text
builtin_interfaces/Time stamp
float32 current_width_mm
float32 current_width_with_offset_mm
float32 fingertip_offset_mm
bool is_moving
bool object_detected
bool s1_pushed
bool s1_trigged
bool s2_pushed
bool s2_trigged
bool safety_error
bool ok