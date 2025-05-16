# 🐞 Bug 2 Algorithm Webots Bot
![Bug 2 Header](https://github.com/dmytro-varich/Bug2-Algorithm-Webots-Bot/blob/main/assets/Bug2_Project_Header.png)
This project demonstrates the implementation of classical [Bug 2](https://medium.com/@sefakurtipek/robot-motion-planning-bug-algorithms-34cf5175ab39) navigation algorithms in a simulated environment using Webots. It includes the development of a custom robot model and a designed simulation environment with obstacles. This project was completed as part of the **Intelligent Robotics** course at [TUKE](https://www.tuke.sk/).

## 🗂️ Project Structure
```
Bug2-Algorithm-Webots-Bot/
├── controllers/
│   └── bug_2_controller/
│       ├── bug_2_controller.py        # Main control script implementing Bug2 algorithm
│       ├── initialization.py          # Initialization of sensors, motors, and robot parameters
│       └── navigation_utils.py        # Utility functions for navigation logic and state handling
├── protos/
│   ├── boxObject.proto                # Custom obstacle/object prototype
│   └── robotModel.proto               # Custom robot model definition
├── worlds/
│   └── world.wbt                      # Webots world file with robot and obstacles
```

## 🛠️ Tech Stack:
- Language: `Python`
- Environment: `Webots`
- Robot Model: `Custom`
- Algorithm: `Bug2`
  
## 🎥 Demo Video
https://github.com/user-attachments/assets/14758d53-76c9-48fe-9161-3371dce5e57a

## ▶️ How to run
1. Install [Webots](https://cyberbotics.com/).

2. Clone the repository:

   ```bash
   git clone https://github.com/dmytro-varich/Bug2-Algorithm-Webots-Bot.git
   ```

3. Open the `world.wbt` file in Webots.

4. Start the simulation.

5. The robot will begin moving toward the goal, navigating around obstacles using the Bug2 algorithm.

## 🔁 Algorithm
![Bug 2 Scheme](https://github.com/dmytro-varich/Bug-Algorithm-Webots-Bot/blob/main/assets/Bug_2_Scheme.png)

The **Bug2 algorithm** is a classical path-planning algorithm for mobile robots navigating in environments with unknown obstacles. It combines **goal-oriented motion** with **obstacle following**, making it both simple and effective for many robotic applications. Bug2 allows a robot to move towards a goal in a straight line (called the **M-line**) until it encounters an obstacle. When that happens, the robot follows the obstacle’s boundary until it can **rejoin the M-line** and continue toward the goal.

## 🤖 Robot Model

<p align="center">
  <img src="https://github.com/dmytro-varich/Bug2-Algorithm-Webots-Bot/blob/main/assets/robot_model.gif" alt="Robot Model 3D view" />
</p>

The robot model in Webots represents a mobile platform with a differential drive and a set of sensors for navigation and spatial orientation. The robot is equipped with various sensors, including **lidars**, a **compass**, **GPS**, and **ultrasonic sensors (sonars)**, enabling obstacle detection and environmental awareness. Each wheel is controlled by a **motor** and tracked with a **position encoder** to enable precise motion control. Both the wheels and the robot body have `boundingObjects` defined via `Shape` (Cylinder). A simplified physical behavior is specified (**mass**, **density**) suitable for motion simulation.
<details>
  <summary><b>Drives and Wheels</b></summary>

| Wheel        | Position       | Mass | Size           | Motor         |
| ------------ | -------------- | ---- | -------------- | ------------- |
| `wheel_left`  | Rear left      | 1 kg | r=0.05, h=0.03 | `motor_left`  |
| `wheel_right` | Rear right     | 1 kg | r=0.05, h=0.03 | `motor_right` |
| `wheel_rear`  | Support (rear) | 5 kg | r=0.05, h=0.04 | `motor_rear`  |
</details>

<details>
  <summary><b>Lidars</b></summary>

| Name          | Position      | Field of View | Range | Resolution |
| ------------- | ------------- | ------------- | ----- | ---------- |
| `lidar_front` | Front         | ~60°          | 0.5 m | 256        |
| `lidar_left`  | Left          | ~45°          | 0.6 m | 256        |
| `lidar_right` | Right (fixed) | ~45°          | 0.6 m | 256        |
</details>

<details>
  <summary><b>Ultrasonic Distance Sensors (Sonars)</b></summary>

| Name               | Position   | Direction          | Max Range |
| ------------------ | ---------- | ------------------ | --------- |
| `sonar_front`       | Front      | Forward            | 0.5 m     |
| `sonar_right`       | Right      | Right              | 0.5 m     |
| `sonar_back_right`  | Back right | Back-right (~30°)  | 0.5 m     |
| `sonar_left`        | Left       | Left               | 0.5 m     |
| `sonar_back_left`   | Back left  | Back-left (~30°)   | 0.5 m     |
</details>

<details>
  <summary><b>Additional Sensors</b></summary>

* **GPS** — determines the global coordinates of the robot.  
* **Compass** — measures orientation relative to magnetic north.
</details>

## ⚙️ Implementation
![Bug 2 Algorithm](https://github.com/dmytro-varich/Bug-Algorithm-Webots-Bot/blob/main/assets/Bug2_Algorithm.drawio.png)

The algorithm is implemented in **Python** using the **Webots** simulation environment with a custom robot model. The navigation logic is based on two main states: `go_to_goal` (moving towards the target) and `follow_wall` (obstacle avoidance). The robot relies on **GPS**, **compass**, **lidar**, and **ultrasonic sensors** to perceive its environment and make decisions.

At the start, the robot determines its initial position and begins moving in a straight line towards the goal (the so-called **M-line**). If it encounters an obstacle, it switches to the `follow_wall` state, where it follows the boundary of the obstacle. The robot records the **hit point** (where it first touches the obstacle) and searches for a **leave point** on the M-line that is closer to the goal.

While following the wall, the robot transitions between substates:

* `turn_right` — turning right when an obstacle is directly in front.
* `turn_left` — turning left if the wall is lost on the left side.
* `move_forward` — moving forward along the wall.
* `search_wall` — searching for the wall after a turn.

When the robot returns to the M-line and is closer to the goal than it was at the hit point, it switches back to `go_to_goal` and continues moving towards the target. The implementation also includes a safeguard against infinite loops: if the robot returns to the hit point on the M-line and the goal is still unreachable, it stops and concludes that the goal cannot be reached.

## 🧛🏻 Author
Dmytro Varich is the creator of this robotics project. You can learn more about his projects on his personal [Telegram channel](https://t.me/varich_channel), as well as connect with him via LinkedIn ([dmytro-varich](https://www.linkedin.com/in/dmytro-varich/)) and email (varich.it@gmail.com).
