
<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue?logo=ros" />
  <img src="https://img.shields.io/badge/Python-3.10-yellow?logo=python" />
  <img src="https://img.shields.io/badge/License-Apache%202.0-blue.svg?logo=apache" />
</p>

# LoBotHouse (Implementation of a Robotic Automation Process System Using ROS2)  

**A collaborative robot (cobot) for construction that supports labor-intensive tasks and mitigates high accident risks on site** **from Doosan Robotics Rokey Bootcamp3 in 2025**

---

## 🗂️ Overview
This project utilizes the Doosan collaborative robot M0609 to symbolically implement and prototype key construction functions commissioned by government or enterprise clients. The robot interprets architectural blueprints, performs construction tasks accordingly, and detects defective materials, offering a proof-of-concept system for automated building assistance.

---

## 🛠️ Equipment and Materials Used
![Equipment and Materials](image/materials.png)
- Doosan Collaborative Robot M0609  
- ROS2 Humble + Ubuntu 22.04

---

## Scenario
1. The M0609 robot reads a blueprint for the building to be constructed.
2. It interprets the blueprint by measuring the height of steel bars embedded in the plate, saving information about the building type and its position.
3. The robot transports construction materials for foundation work and carries out concrete pouring.
4. Based on the interpreted data, the robot stacks LEGO-like blocks to build the structure.
- If the gripper detects an abnormal object, the robot moves it to the disposal area.
- Even if the robot stops due to an error or emergency, it can resume operation using system variables.
