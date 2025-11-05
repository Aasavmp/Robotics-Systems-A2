This repo looks at development of code to control a 3PI+ robot, create a test environment and run experiments.


# Optimising Robotic Search Algorithms  
### Balancing Speed, Accuracy, and Thoroughness in Autonomous Navigation Systems

**Authors:** Aasav Patel & Frank Panton  
**Full Report:** [Robotic_Systems_A2_Report.pdf](./Assessment%202%20v1/Submission%20folder/Robotic_Systems_A2_Report.pdf)

---

## Overview

This project investigates how search algorithm design affects **speed**, **accuracy**, and **thoroughness** in autonomous navigation. Using the Pololu 3Pi+ robot, various wave-based search strategies were tested to determine optimal configurations for different operational goals.

This work builds upon an earlier branch of the project where **complete odometry kinematics** and **PID control loops** for both **speed** and **directional heading** were developed and tuned. These systems provided the foundation for accurate motion control and consistent waypoint detection in the experiments that follow.

---

## Experimental Setup

### Search Patterns
The robot was programmed to follow **sine** and **square** wave search paths at different wavelengths (200 mm and 400 mm) and speeds (1–3).

![Search Algorithms](./Assessment%202%20v1/SearchAlgorithms%20Plot%20v2.jpg)

### Line Sensor Geometry
Each line sensor’s offset from the robot centre was accounted for to improve waypoint localisation accuracy.

![Line Sensor Schematic](./Assessment%202%20v1/Line%20Sensor%20schmatics.png)

### Test Environment
Two A3 sheets with 10 random black waypoints each were used for testing consistency.

![Waypoint Picture](./Assessment%202%20v1/Waypoint%20Picture.png)

---

## Key Metrics

- **Thoroughness:** Number of waypoints found  
- **Accuracy:** Distance between detected and true waypoint coordinates  
- **Efficiency:** Time taken per waypoint  

### Example Calculations
- Average absolute error between measured and true coordinates  
- Time per waypoint derived from total search duration and waypoints found  

---

## Results Summary

### Thoroughness
Shorter wavelengths increased coverage, with square waves achieving slightly higher median waypoint counts.

![Number of Waypoints](./Assessment%202%20v1/No%20of%20waypoints.png)

### Accuracy
Longer wavelengths improved positional accuracy. Square waves (400 mm) achieved the lowest error (~9 mm).

![Accuracy Plot](./Assessment%202%20v1/Accuracy%20Plot.png)  
![Distance to Waypoints](./Assessment%202%20v1/distance%20to%20waypoint.png)

Kinematic errors accumulated over time—first waypoints were consistently more accurate than the last.

![Distance to First and Last Waypoints](./Assessment%202%20v1/Distance%20to%20First%20and%20Last%20waypoints.png)

### Efficiency
Higher speeds significantly reduced time per waypoint without major accuracy loss.

![Time per Waypoint](./Assessment%202%20v1/time%20per%20waypoint.png)

### Combined Trade-offs
A clear trade-off emerged between accuracy and thoroughness:  
- Short wavelengths → more waypoints, less accuracy  
- Long wavelengths → fewer waypoints, higher accuracy  

![Waypoint Error vs Number of Waypoints](./Assessment%202%20v1/Waypoint%20error%20vs%20number%20of%20waypoints.png)

---

## Optimal Parameters

| Metric             | Wavelength | Speed | Pattern     |
|--------------------|-------------|--------|--------------|
| **Thoroughness**   | 200 mm      | 3      | Square/Sine  |
| **Accuracy**       | 400 mm      | 3      | Square       |
| **Time/Waypoint**  | 200 mm      | 3      | Sine         |

---

## Conclusions

- **Square waves** are best for accuracy and consistency.  
- **Sine waves** are more time-efficient.  
- **Shorter wavelengths** improve coverage but reduce accuracy.  
- **Higher speeds** yield faster and sometimes more accurate results than expected.

This demonstrates that the ideal parameters depend on task priorities—coverage vs precision.

---

## Future Work

1. Incorporate coloured or greyscale waypoint differentiation.  
2. Expand algorithm library with AI-based searches.  
3. Introduce obstacles to simulate real-world navigation.  
4. Explore swarm robotics and multi-agent communication.

---

## References

1. Basilico, N. *Recent Trends in Robotic Patrolling.* Current Robotics Reports, 2022.  
2. Fountas, S. *Agricultural Robotics for Field Operations.* Sensors, 2020.  
3. Huo, J. *Autonomous Search of Radioactive Sources Through Mobile Robots.* Sensors, 2020.  
4. Fedorenko, G. *Robotic-Biological Systems for Explosive Ordnance Detection.* 2023.  
5. Din, A. *Swarm Robotic Search and Rescue Using Fuzzy Controller.* Computers & Electrical Engineering, 2018.  
6. Pololu Corporation. *Pololu 3Pi+ 32U4 User’s Guide.* 2022.  
7. Somwanshi, D. *Comparison of Fuzzy-PID and PID Controller for DC Motor Speed Control.* 2019.  
8. Hawkes, N. *Robotic Inspection for the Nuclear Industry.* 2020.

---

> 📘 **Full Report:** [Robotic_Systems_A2_Report.pdf](./Assessment%202%20v1/Submission%20folder/Robotic_Systems_A2_Report.pdf)
