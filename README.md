# Vision-Language-Action Model for Robot Control

A Vision-Language-Action (VLA) framework for controlling the UFACTORY xArm 5 DoF robot using visual perception and natural-language instructions.

The system uses a YOLOv8 segmentation model to identify and locate objects within the environment, a Qwen2.5-3B-Instruct large language model to interpret user instructions and generate robot actions, and ROS 2 with MoveIt to execute these actions using the xArm 5.

The current implementation is demonstrated using a food preparation environment in Gazebo Fortress.

## Prerequisites

The project was developed using:

- ROS 2 Humble
- Gazebo Fortress
- UFACTORY xArm ROS 2 packages
- MoveIt
- llama.cpp
- Qwen2.5-3B-Instruct-GGUF

### ROS 2 Humble

Installation instructions:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

### Gazebo Fortress

Installation instructions:

https://gazebosim.org/docs/fortress/install_ubuntu/

### UFACTORY xArm ROS 2

The xArm ROS 2 packages can be found at:

https://github.com/xArm-Developer/xarm_ros2/tree/humble

### Gazebo ROS Packages

https://classic.gazebosim.org/tutorials?tut=ros2_installing

## Installation

Clone this repository into the `src` directory of your ROS 2 workspace containing the xArm ROS 2 packages.

For example:

```bash
cd ~/dev_ws/src
git clone https://github.com/SamDavey02/vlam.git
```

The workspace should then contain both the xArm ROS 2 packages and the `valm` package.

Build the package:

```bash
cd ~/dev_ws
colcon build --packages-select valm
```

Source the workspace:

```bash
source install/setup.bash
```

If this does not work, source ROS 2 first:

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/install/setup.bash
```

## Large Language Model

The language component requires a locally running LLM server on port `8080`.

For this project, the Qwen2.5-3B-Instruct-GGUF model was used:

https://huggingface.co/Qwen/Qwen2.5-3B-Instruct-GGUF

The `Q4_K_M` quantised version was run using `llama.cpp`.

## Running the VLA System

The system requires multiple terminals. Each terminal should have the ROS 2 workspace sourced before running ROS commands.

### Terminal 1 — Start the Gazebo Simulation

```bash
cd ~/dev_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch valm xarm5_simulation.launch.py
```

This launches the xArm 5 and food preparation environment in Gazebo.

### Terminal 2 — Start the LLM Server

```bash
cd ~/llama.cpp

./build/bin/llama-server \
  -hf Qwen/Qwen2.5-3B-Instruct-GGUF:Q4_K_M \
  -ngl 99 \
  -c 4096 \
  --host 127.0.0.1 \
  --port 8080
```

The LLM server must be running before sending instructions to the language component.

### Terminal 3 — Start the Language Planner

```bash
cd ~/dev_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run valm language
```

The language planner receives the current scene state and natural-language instruction before generating a structured robot action plan.

### Terminal 4 — Start the Action Executor

```bash
cd ~/dev_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run valm action_executor
```

The action executor converts the generated action plan into robot movements using the xArm 5 and MoveIt.

### Terminal 5 — Send a User Instruction

Natural-language commands can be sent through the `/user_instruction` ROS 2 topic.

For example:

```bash
ros2 topic pub --once /user_instruction std_msgs/msg/String \
"{data: 'pick up the tomato'}"
```

The VLA system will interpret the command, identify the requested object and generate the actions required to perform the task.

## Example Commands

Other instructions can be tested by changing the text sent to `/user_instruction`.

```bash
ros2 topic pub --once /user_instruction std_msgs/msg/String \
"{data: 'grab the cheese'}"
```

```bash
ros2 topic pub --once /user_instruction std_msgs/msg/String \
"{data: 'move up 5 cm'}"
```

```bash
ros2 topic pub --once /user_instruction std_msgs/msg/String \
"{data: 'place the tomato on the bread'}"
```

## Future Work

### Updating the Segmentation Model

To update the segmentation model, place the new trained model inside:

```text
valm/training_runs/
```

Then update the model path in:

```text
valm/valm/vision.py
```

The model is currently loaded around line 51 using:

```python
# Load trained YOLO segmentation model
self.model = YOLO('/home/sam/dev_ws/src/valm/training_runs/food_seg_v5_sim/weights/best.pt')
```

Replace this path with the location of the new model weights.

For example:

```python
self.model = YOLO('/home/sam/dev_ws/src/valm/training_runs/new_model/weights/best.pt')
```

### Updating the Simulation Environment

Gazebo world files are stored in:

```text
valm/worlds/
```

Custom Gazebo object models are stored in:

```text
valm/models/
```

To use a different simulation world, update the selected world in:

```text
valm/launch/xarm5_simulation.launch.py
```

The world is currently selected around line 39 using:

```python
# Choose the world
'world': PathJoinSubstitution([FindPackageShare('valm'),'worlds','vision_test.world']),
```

Replace `vision_test.world` with the name of the new world file.

For example:

```python
'world': PathJoinSubstitution([FindPackageShare('valm'),'worlds','new_environment.world']),
```

This allows the system to be extended with new vision models, objects, and simulated environments without changing the overall VLA architecture.

## System Overview

The VLA system follows three main stages:

1. **Vision** — YOLOv8 segmentation identifies objects and calculates their position and grasp information.
2. **Language** — Qwen2.5-3B-Instruct interprets the user's command and current scene state to generate structured robot actions.
3. **Action** — The action executor converts the generated actions into xArm 5 movements using ROS 2 and MoveIt.

The modular architecture allows the vision and language models to be replaced or improved independently.
