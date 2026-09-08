# 2D LIDAR Sensor Modelling and Simulation

## Overview
This project features a mathematical model and an interactive numerical simulation of a two-dimensional (2D) rotating LIDAR (Light Detection and Ranging) sensor [cite: 1]. It is designed to demonstrate mobile robot perception [cite: 1]. The sensor is modelled using a physics-based range equation combined with ray-casting to determine line-of-sight (LOS) visibility [cite: 1]. Additionally, it features a probabilistic measurement model that includes additive Gaussian noise and stochastic dropout to simulate adverse environmental conditions like fog or rain [cite: 1]. 

## Files in this Repository
* **`LIDAR_Project_Report.pdf`**: A detailed project report presenting the mathematical models, numerical simulation details, and validation experiments showing how measurements depend on sensor state, target state, and environmental conditions [cite: 1].
* **`lidarrun.py`**: An interactive, live Python simulator that continuously scans a bounded room [cite: 2]. It provides real-time range-vs-angle plots and allows you to dynamically alter the sensor's environment and operating conditions [cite: 2].

## Requirements
To run the interactive simulator, you need Python installed along with the `pygame` and `numpy` libraries [cite: 2]. 
You can install the dependencies using pip:
```bash
pip install pygame numpy
```

## How to Run
Run the simulator from your terminal or command prompt by executing [cite: 2]:
```bash
python lidarrun.py
```

## Interactive Controls
Once the simulator is running, you can interact with the environment and the sensor using the following controls [cite: 2]:

### Mouse Controls:
* **Left click**: Place a square obstacle at the mouse position [cite: 2].
* **Right click**: Remove the nearest obstacle to the mouse [cite: 2].

### Keyboard Controls:
* **Arrow keys**: Move the sensor (Up/Down/Left/Right) [cite: 2].
* **A / D**: Rotate the sensor left / right [cite: 2].
* **F**: Cycle field of view (90 / 180 / 270 / 360 deg) [cite: 2].
* **+ / -**: Increase / decrease measurement noise (sigma) [cite: 2].
* **[ / ]**: Decrease / increase dropout probability [cite: 2].
* **R**: Reset (clear all obstacles) [cite: 2].
* **ESC**: Quit the simulation (can also close the window) [cite: 2].
