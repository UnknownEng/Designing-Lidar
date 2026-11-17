# Designing LIDAR — 2D LIDAR Sensor Modelling and Simulation

An interactive, real-time simulator of a **2D rotating LIDAR sensor**, built with Python, NumPy and Pygame.
Drive the sensor around a room, drop obstacles with the mouse, and watch how every range measurement
responds to the **sensor state**, the **environment**, and the **operating conditions** (noise and dropout), live.

It is designed as a teaching and experimentation tool for **mobile-robot perception**: every effect in the
mathematical model below can be switched on, tuned, and seen immediately on three live graphs.

## Features
- **Physics-based ray casting:** 160 beams tested against every wall and obstacle edge in a single vectorized NumPy pass, so it stays smooth with dozens of obstacles.
- **Probabilistic sensor model:** Gaussian range noise and random beam dropout, both adjustable while it runs.
- **Configurable field of view:** 90°, 180°, 270° or full 360°.
- **Interactive world:** place and remove square obstacles with the mouse. The sensor collides with them instead of passing through.
- **Three live graphs:**
  1. Range vs. beam angle
  2. Minimum range over time
  3. Histogram of measured ranges
- **Live readouts:** pose, heading, FOV, FPS and min/max/mean range.

## Mathematical Model
- **Sensor geometry & state:** The sensor emits $n_b$ discrete beams uniformly spaced across a field of view of width $\Phi$. The beams translate and rotate rigidly with the sensor state $x = (x_s, y_s, \theta_s)$.
- **Line-of-sight ray casting:** The environment is a set of line segments (walls and obstacle edges). Each beam's true range is the distance to the closest valid ray–segment intersection in front of the sensor, found by solving a 2×2 linear system and clipped at the maximum range $r_{max}$.
- **Probabilistic measurements:** The operating conditions $\Lambda = (\sigma, p_{drop})$ govern how measurements degrade:
  - **Gaussian noise:** Each valid return is corrupted by additive noise $w_i \sim \mathcal{N}(0, \sigma^2)$. Beams that hit nothing stay at $r_{max}$.
  - **Stochastic dropout:** To mimic fog, rain or low-reflectivity surfaces, each beam fails with probability $p_{drop}$ and reports $r_{max}$.

## Getting Started
```bash
pip install -r requirements.txt
python lidarrun.py
```
Requires Python 3.8+.

## Controls
| Input | Action |
|---|---|
| Left click | Place a square obstacle |
| Right click | Remove the nearest obstacle |
| Arrow keys | Move the sensor |
| A / D | Rotate the sensor left / right |
| F | Cycle field of view (90 / 180 / 270 / 360°) |
| + / − | Increase / decrease noise σ |
| [ / ] | Decrease / increase dropout probability |
| R | Clear all obstacles |
| Esc | Quit |

## Default Parameters
| Parameter | Value |
|---|---|
| Room size | 10 m × 10 m |
| Number of beams $n_b$ | 160 |
| Max range $r_{max}$ | 7 m |
| Noise σ | 0.02 m (0 – 0.5) |
| Dropout $p_{drop}$ | 0.01 (0 – 0.9) |
| Frame rate | 30 FPS |

All values are constants at the top of `lidarrun.py` and are easy to change.

## Repository Structure
```
Designing-Lidar/
├── lidarrun.py        # interactive simulator (model + visualisation)
├── requirements.txt   # Python dependencies
└── README.md
```
