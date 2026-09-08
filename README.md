# 2D LIDAR Sensor Modelling and Simulation

## Overview
This project features a mathematical model and an interactive numerical simulation of a two-dimensional (2D) rotating LIDAR (Light Detection and Ranging) sensor[cite: 1]. It is designed to demonstrate mobile robot perception by showing how measurements depend on sensor state, target state, and operating conditions[cite: 1]. 

## Technical & Mathematical Model
The simulation mathematically models the sensor's physical and probabilistic characteristics[cite: 1]:
* **Sensor Geometry & State:** The sensor is modelled as $n_b$ discrete angular beams uniformly spaced across a field of view of angular width $\Phi$[cite: 1]. The field of view and beam origins translate and rotate rigidly based on the sensor state $x = (x_s, y_s, \theta_s)$[cite: 1].
* **Physics-Based Ray-Casting (LOS):** The environment is represented as line segments (walls and edges)[cite: 1]. Line-of-sight visibility is calculated mathematically by solving linear systems for ray-segment intersections[cite: 1]. The true range is the distance to the closest valid intersection in front of the sensor, clipped at the sensor's maximum range $r_{max}$[cite: 1].
* **Probabilistic Measurements:** Operating conditions $\Lambda = (\sigma, p_{drop})$ govern the measurement degradation[cite: 1]:
    * **Gaussian Noise:** Each valid beam return is corrupted by additive Gaussian noise $w_i \sim N(0, \sigma^2)$ to model standard sensor inaccuracy[cite: 1].
    * **Stochastic Dropout:** To simulate adverse weather (like fog or rain) or low-reflectivity surfaces, each beam has a probability $p_{drop}$ of failing to register a return, defaulting instead to $r_{max}$[cite: 1].

## Files in this Repository
* **`LIDAR_Project_Report.pdf`**: A detailed project report presenting the mathematical models, numerical simulation details, and validation experiments[cite: 1].
* **`lidarrun.py`**: An interactive, live Python simulator that continuously scans a bounded room[cite: 2]. It provides real-time range-vs-angle plots and allows you to dynamically alter the sensor's environment and operating conditions[cite: 2].

## Requirements
To run the interactive simulator, you need Python installed along with the `pygame` and `numpy` libraries[cite: 2]. 
You can install the dependencies using pip:
`pip install pygame numpy`

## How to Run
Run the simulator from your terminal or command prompt by executing[cite: 2]:
`python lidarrun.py`

## Interactive Controls
Once the simulator is running, you can interact with the environment and the sensor using the following controls[cite: 2]:

### Mouse Controls
* **Left click**: Place a square obstacle at the mouse position[cite: 2].
* **Right click**: Remove the nearest obstacle to the mouse[cite: 2].

### Keyboard Controls
* **Arrow keys**: Move the sensor (Up/Down/Left/Right)[cite: 2].
* **A / D**: Rotate the sensor left / right[cite: 2].
* **F**: Cycle field of view (90 / 180 / 270 / 360 deg)[cite: 2].
* **+ / -**: Increase / decrease measurement noise ($\sigma$)[cite: 2].
* **[ / ]**: Decrease / increase dropout probability ($p_{drop}$)[cite: 2].
* **R**: Reset (clear all obstacles)[cite: 2].
* **ESC**: Quit the simulation (can also close the window)[cite: 2].
