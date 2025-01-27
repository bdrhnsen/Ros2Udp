# ROS 2 UDP Nodes

This repository contains two ROS 2 nodes designed for UDP communication. These nodes facilitate data exchange between systems using the User Datagram Protocol (UDP). The implementation includes a listener node (`ros2_udp_listener`) and a sender node (`ros2_udp_sender`).

## Features

### ros2_udp_listener
- Listens for UDP packets on a specified port.
- Parses incoming data and publishes it as ROS 2 messages.
- Supports structured observation data, including ego vehicle position, speed, and surrounding vehicles' information.

### ros2_udp_sender
- Sends ROS 2 messages as UDP packets to a specified IP and port.
- Encodes messages into a compact binary format for efficient transmission.

## Folder Structure
```plaintext
./
    ros2_udp_listener/
        Dockerfile
        src/
            udp_listener/
                package.xml
                CMakeLists.txt
                src/
                    udp_listener_node.cpp
    ros2_udp_sender/
        Dockerfile
        src/
            udp_sender/
                package.xml
                CMakeLists.txt
                src/
                    udp_sender_node.cpp
```

## Prerequisites
- **Docker** installed on your machine.
- **ROS 2 Humble** (if running without Docker).

## Setup

### Build the Nodes
#### Using Docker
1. Navigate to the directory containing the Dockerfile:
   ```bash
   cd ros2_udp_listener
   docker build -t ros2_udp_listener .
   ```
   Repeat for `ros2_udp_sender`.

2. Run the containers:
   ```bash
   docker run -it ros2_udp_listener
   docker run -it ros2_udp_sender
   ```

#### Using Colcon
1. Source your ROS 2 Humble installation:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```
3. Source the workspace:
   ```bash
   source install/local_setup.bash
   ```

## Usage

### ros2_udp_listener
The `ros2_udp_listener` listens for UDP packets on port **5005** and publishes received data as ROS 2 messages on the topic `/udp_observation`.

Run the node:
```bash
ros2 run udp_listener udp_listener_node
```

### ros2_udp_sender
The `ros2_udp_sender` subscribes to the topic `/vehicle/cmd` and sends data as UDP packets to the destination IP **172.18.0.2** on port **6005**.

Run the node:
```bash
ros2 run udp_sender udp_sender_node
```

### Example Communication Flow
1. Run the listener node to start receiving UDP messages.
2. Publish a `geometry_msgs/Twist` message to `/vehicle/cmd` to send a UDP packet using the sender node:
   ```bash
   ros2 topic pub /vehicle/cmd geometry_msgs/Twist "{ linear: {x: 1.0}, angular: {z: 0.5} }"
   ```
3. Observe the listener node output showing the received data.

## Node Details

### UDP Listener Node
- **Source File:** `ros2_udp_listener/src/udp_listener/src/udp_listener_node.cpp`
- **Publishes To:** `/udp_observation`
- **Data Structure:**
  - Ego vehicle position (x, y) and speed.
  - Position and speed of up to 5 surrounding vehicles.

### UDP Sender Node
- **Source File:** `ros2_udp_sender/src/udp_sender/src/udp_sender_node.cpp`
- **Subscribes To:** `/vehicle/cmd`
- **Sends To:** IP `172.18.0.2`, Port `6005`
- **Message Format:**
  - Double values for steering and acceleration packed into 16 bytes.

