# Chariot Sensor Selection for Cooperative Obstacle Detection and Removal in a Swarm Robot System

## Abstract

This literature review investigates the most suitable sensor configuration for a low-cost swarm robotics system designed to detect and remove obstacles from roads. The project scenario consists of multiple small Chariot robots that collaboratively detect and push obstacles away from a roadway. If the Chariot swarm cannot remove the obstacle, a secondary robot type, referred to as the Bulldozer, is summoned to clear the road. The robots operate outdoors and communicate through MQTT while using BLE beacons for localization support within a Webots simulation environment.

The research evaluates several sensing technologies, including ultrasonic sensors, infrared sensors, camera systems, LiDAR, and Time-of-Flight (ToF) sensors. The comparison focuses on cost, accuracy, size, computational requirements, simulation compatibility, and practical feasibility for small robotic platforms. Based on the reviewed literature and project constraints, Time-of-Flight sensors are identified as the most suitable solution due to their compact size, low cost, sufficient range, and ease of integration in both hardware and Webots simulations.

---

# 1. Introduction

Swarm robotics is an emerging field in which multiple robots cooperate to complete tasks that would be difficult or impossible for a single robot. Swarm systems are commonly inspired by natural collective behavior such as ant colonies or bee swarms. These systems are especially useful for applications involving distributed sensing, cooperative transport, search and rescue, and autonomous infrastructure maintenance [5].

This project focuses on a swarm-based road-clearing system. The primary objective is to develop a group of small autonomous Chariot robots capable of traversing roads, detecting obstacles, triangulating their positions, avoiding collisions, and collaboratively removing obstacles from the roadway. When the Chariot robots are unable to remove an obstacle due to size or weight limitations, a specialized Bulldozer robot is summoned to perform the task.

The project is simulated in Webots and uses MQTT for communication between robots. BLE beacons are used to support localization and triangulation. One of the most important design decisions in this project is the selection of suitable sensors for obstacle detection and localization.

This literature review investigates which sensor configuration is most suitable for the project while considering budget limitations, hardware size constraints, simulation feasibility, and system complexity.

---

# 2. Project Context

## 2.1 Chariot Swarm Scenario

The system consists of two robot types:

### Chariot Robots

The Chariot robots are lightweight autonomous swarm robots responsible for:
- traversing roads,
- detecting obstacles,
- avoiding collisions,
- triangulating their positions,
- and collaboratively pushing obstacles.

### Bulldozer Robot

The Bulldozer robot is a heavier support robot that is summoned when the Chariot swarm cannot remove an obstacle independently.

---

## 2.2 Functional Requirements

### Functional Swarm Requirements
- Traverse the road
- Detect obstacles
- Triangulate position
- Avoid collisions

### Chariot Requirements
- Summon the Bulldozer

### Bulldozer Requirements
- Remove obstacles from the road

---

## 2.3 Simulation Context

The project is developed using:
- Webots simulation software,
- MQTT communication,
- BLE beacons for localization support.

Webots is particularly suitable because it supports realistic robot physics and multiple sensor types, including distance sensors that can emulate ToF sensors [3].

---

# 3. Methodology

The literature review was conducted by comparing multiple sensing technologies commonly used in autonomous robotics systems. The selected technologies were evaluated based on:
- obstacle detection capability,
- localization support,
- computational requirements,
- cost,
- physical size,
- outdoor usability,
- and compatibility with Webots simulations.

Scientific literature, technical datasheets, robotics textbooks, and official documentation were used as sources. The goal of the comparison was to identify the most realistic and achievable sensor configuration for a small swarm robot platform operating within budget and hardware limitations.

---

# 4. Sensor Requirements

## 4.1 Obstacle Detection Requirements

The selected sensor system must:
- detect static road obstacles,
- operate outdoors,
- function on a small robot platform,
- provide sufficient range for collision avoidance,
- support multiple robots operating simultaneously.

---

## 4.2 Localization Requirements

The swarm must estimate robot positions using BLE beacon triangulation. Localization does not need centimeter-level precision but should provide enough positional awareness for:
- swarm coordination,
- navigation,
- and obstacle handling.

---

## 4.3 Physical and Budget Constraints

The Chariot platform has several constraints:
- limited mounting space,
- limited power availability,
- limited processing power,
- and a restricted budget.

These constraints eliminate large or expensive sensor systems such as advanced LiDAR units.

---

# 5. Literature Review

## 5.1 Ultrasonic Sensors

Ultrasonic sensors are commonly used for obstacle detection in low-cost robotics systems. These sensors measure distance by transmitting ultrasonic sound waves and calculating the time required for the echo to return.

### Advantages
- Low cost
- Simple implementation
- Widely available
- Reliable on many surfaces

### Disadvantages
- Wide sensing cone causes inaccurate readings
- Interference between multiple robots
- Lower precision compared to optical sensors
- Bulky relative to the Chariot size

Ultrasonic sensors are useful for simple robotics applications but are less suitable for dense swarm systems where multiple robots operate close together.

---

## 5.2 Infrared Sensors

Infrared (IR) sensors detect reflected infrared light to estimate distance.

### Advantages
- Very low cost
- Compact size
- Easy implementation

### Disadvantages
- Sensitive to lighting conditions
- Poor outdoor performance
- Limited range
- Surface color affects readings

Because the robots operate outdoors, IR sensors are considered unreliable for this project.

---

## 5.3 Camera-Based Detection

Camera systems are widely used in autonomous robotics for object recognition and navigation.

### Advantages
- Rich environmental information
- Supports advanced AI and computer vision
- Can classify obstacles

### Disadvantages
- High computational requirements
- Increased software complexity
- Sensitive to weather and lighting
- Difficult to implement within project scope

Although camera systems are powerful, they introduce unnecessary complexity for the goals of this project.

---

## 5.4 LiDAR

LiDAR sensors create environmental maps by measuring distances using laser pulses.

### Advantages
- High accuracy
- Excellent obstacle detection
- Large sensing range

### Disadvantages
- Expensive
- Large physical size
- High power consumption
- Difficult to mount on small robots

LiDAR provides excellent performance but is unrealistic for a small low-cost Chariot platform.

---

## 5.5 Time-of-Flight Sensors

Time-of-Flight (ToF) sensors determine distance by measuring the travel time of emitted light pulses.

Popular examples include:
- VL53L0X
- VL53L1X

### Advantages
- Compact size
- Low power consumption
- Low cost
- Accurate short-range detection
- Easy Webots simulation
- Fast measurement rate

### Disadvantages
- Limited field of view
- Limited range compared to LiDAR

The VL53L1X is particularly suitable because it offers longer range and improved accuracy compared to the VL53L0X [1][2].

ToF sensors provide the best balance between:
- size,
- cost,
- simplicity,
- and detection performance.

---

## 5.6 BLE Beacon Localization

BLE beacons can support indoor and outdoor positioning using RSSI signal strength measurements.

### Advantages
- Low cost
- Lightweight communication
- Easy integration
- Compatible with swarm systems

### Disadvantages
- Low positional accuracy
- Signal noise
- Outdoor interference
- RSSI instability

BLE localization is suitable for approximate swarm positioning but should not be relied upon for precise navigation.

---

# 6. Comparison Matrix

| Sensor Type | Cost | Size | Accuracy | Outdoor Performance | Complexity | Suitability |
|---|---|---|---|---|---|---|
| Ultrasonic | Low | Medium | Medium | Medium | Low | Moderate |
| Infrared | Very Low | Small | Low | Poor | Low | Poor |
| Camera | Medium | Medium | High | Medium | High | Moderate |
| LiDAR | Very High | Large | Very High | Excellent | High | Poor |
| ToF | Low | Small | High | Good | Low | Excellent |
| BLE Beacons | Low | Small | Low | Medium | Low | Support Only |

---

# 7. Sensor Evaluation Scoring

| Sensor | Accuracy | Cost | Size | Complexity | Simulation Support | Final Score |
|---|---|---|---|---|---|---|
| Ultrasonic | 6/10 | 9/10 | 5/10 | 9/10 | 8/10 | 37/50 |
| Infrared | 4/10 | 10/10 | 9/10 | 9/10 | 7/10 | 39/50 |
| Camera | 9/10 | 5/10 | 6/10 | 3/10 | 7/10 | 30/50 |
| LiDAR | 10/10 | 2/10 | 3/10 | 5/10 | 8/10 | 28/50 |
| ToF | 8/10 | 9/10 | 10/10 | 9/10 | 10/10 | 46/50 |

The evaluation indicates that Time-of-Flight sensors provide the best overall balance between performance, affordability, physical size, and implementation complexity.

---

# 8. Recommended Sensor Configuration

Based on the literature review and project constraints, the recommended sensor configuration is:

## Chariot Robot
- Front-facing Time-of-Flight sensors (VL53L1X preferred)
- BLE beacon receiver for localization
- Wheel encoders or IMU if available

## Bulldozer Robot
- Time-of-Flight sensors for navigation
- MQTT communication module
- Obstacle pushing mechanism

This configuration is considered the most realistic and achievable solution within the project scope while maintaining acceptable obstacle detection performance and swarm coordination capabilities.

---

# 9. Motor Recommendation

Besides sensor selection, motor selection is also important for the mobility of the Chariot robots.

The motors should:
- be compact,
- affordable,
- provide sufficient torque,
- support differential drive movement,
- and operate efficiently on battery power.

A realistic recommendation for the Chariot platform is:
- small DC geared motors,
- preferably with wheel encoders,
- operating between 6V and 12V.

Wheel encoders improve odometry and help estimate robot movement between BLE beacon updates. Differential drive systems are commonly used in swarm robotics because they are simple to control and easy to simulate in Webots.

Stepper motors were not recommended because they:
- consume more power,
- are less efficient for mobile robots,
- and increase mechanical complexity.

---

# 10. Webots Implementation Approach

Webots supports several sensor types suitable for this project.

## Obstacle Detection

ToF sensors can be represented using:
- `DistanceSensor` nodes,
- ray-based sensing,
- short-range obstacle detection.

## Localization

BLE beacon positioning can be simulated using:
- predefined beacon nodes,
- signal strength approximations,
- coordinate broadcasting.

## Communication

MQTT communication can be implemented externally or through robot controllers to:
- broadcast obstacle detections,
- coordinate swarm movement,
- summon the Bulldozer robot.

---

# 11. MQTT Communication Design

MQTT is a lightweight publish-subscribe communication protocol suitable for distributed robotics systems [4].

## Advantages
- Lightweight
- Scalable
- Low bandwidth usage
- Easy swarm communication

## Proposed MQTT Topics

| Topic | Purpose |
|---|---|
| `swarm/position` | Share robot position |
| `swarm/obstacle` | Broadcast obstacle detection |
| `swarm/help` | Request Bulldozer support |
| `bulldozer/status` | Bulldozer availability |
| `swarm/navigation` | Share navigation updates |

---

# 12. Conclusion

This literature review investigated sensor options for a low-cost swarm robotics system designed to detect and remove obstacles from roads.

Several sensing technologies were analyzed, including ultrasonic sensors, infrared sensors, cameras, LiDAR, and Time-of-Flight sensors. The analysis considered:
- cost,
- size,
- implementation complexity,
- simulation compatibility,
- and outdoor performance.

The research concludes that Time-of-Flight sensors are the most suitable choice for the Chariot robots. ToF sensors provide accurate short-range obstacle detection while remaining compact, affordable, and easy to integrate into both physical hardware and Webots simulations.

BLE beacons are considered suitable for approximate localization and swarm coordination, while MQTT provides an efficient communication framework for robot cooperation and Bulldozer coordination.

LiDAR and camera-based systems were rejected because they exceed the practical hardware and complexity limitations of the project.

The final proposed architecture balances realism, technical feasibility, cost-efficiency, and implementation simplicity while still achieving the project goals.

---

# References

[1] STMicroelectronics, "VL53L1X Time-of-Flight ranging sensor datasheet," 2023. [Online]. Available: https://www.st.com/

[2] STMicroelectronics, "VL53L0X Time-of-Flight ranging sensor datasheet," 2023. [Online]. Available: https://www.st.com/

[3] Cyberbotics, "Webots Documentation – DistanceSensor," 2024. [Online]. Available: https://cyberbotics.com/

[4] MQTT.org, "MQTT Version 5.0 Specification," 2024. [Online]. Available: https://mqtt.org/

[5] Y. Tan, *Handbook of Research on Design, Control and Modeling of Swarm Robotics*. Hershey, PA, USA: IGI Global, 2019.

[6] R. Murphy, *Introduction to AI Robotics*. Cambridge, MA, USA: MIT Press, 2019.

[7] R. Siegwart, I. Nourbakhsh, and D. Scaramuzza, *Introduction to Autonomous Mobile Robots*, 2nd ed. Cambridge, MA, USA: MIT Press, 2011.

[8] A. Alarifi et al., "Ultra Wideband Indoor Positioning Technologies: Analysis and Recent Advances," *Sensors*, vol. 16, no. 5, pp. 707-728, 2016.

[9] M. Schranz, M. Umlauft, M. Sende, and W. Elmenreich, "Swarm Robotic Behaviors and Current Applications," *Frontiers in Robotics and AI*, vol. 7, pp. 1-16, 2020.