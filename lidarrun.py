"""
lidarrun.py — Interactive Live LIDAR Simulator
=================================================
Opens a live window showing a simulated 2D LIDAR sensor continuously scanning
a room. You can place and remove obstacles with the mouse, move/rotate the
sensor with the keyboard, and watch the live range-vs-angle plot update in
real time on the right side of the window.

CONTROLS
  Left click        : place a square obstacle at the mouse position
  Right click        : remove the nearest obstacle to the mouse
  Arrow keys          : move the sensor (Up/Down/Left/Right)
  A / D               : rotate the sensor left / right
  F                   : cycle field of view (90 / 180 / 270 / 360 deg)
  + / -               : increase / decrease measurement noise (sigma)
  [ / ]               : decrease / increase dropout probability
  R                   : reset (clear all obstacles)
  ESC or close window : quit

HOW TO RUN
  pip install pygame numpy
  python lidarrun.py
"""

import sys
import math
import numpy as np
import pygame

# ----------------------------- Window / world setup -----------------------------
WIN_W, WIN_H = 1200, 830
ROOM_PANEL_W = 700          # left panel: the room / sensor view
PLOT_PANEL_W = WIN_W - ROOM_PANEL_W  # right panel: live range-vs-angle graph

WORLD_SIZE = 10.0           # room is WORLD_SIZE x WORLD_SIZE meters, centered at (0,0)
PIXELS_PER_METER = (ROOM_PANEL_W - 40) / WORLD_SIZE
ORIGIN_PX = (ROOM_PANEL_W // 2, WIN_H // 2)

FPS = 30

# Sensor / LIDAR parameters (FOV, noise sigma, and dropout are all live-adjustable
# at runtime -- see FOV_OPTIONS / key handlers below -- these are just the defaults)
MAX_RANGE = 7.0
FOV_OPTIONS = [90.0, 180.0, 270.0, 360.0]
FOV_DEG_DEFAULT = 360.0
N_BEAMS = 160
NOISE_SIGMA_DEFAULT = 0.02
DROPOUT_DEFAULT = 0.01
HISTORY_LEN = 150   # number of past frames kept for the min-range trend graph
OBSTACLE_HALF = 0.35  # half side length of a placed square obstacle (m)
SENSOR_RADIUS = 0.15  # collision radius of the sensor body (m)

# Colors
BLACK = (15, 15, 20)
WHITE = (235, 235, 235)
GRAY = (90, 90, 100)
ORANGE = (255, 170, 60)
RED = (235, 60, 60)
BLUE = (70, 140, 255)
GREEN = (80, 220, 120)
PANEL_BG = (25, 25, 32)


def world_to_px(x, y):
    """Convert world meters (x right, y up) to screen pixels (y down)."""
    px = ORIGIN_PX[0] + x * PIXELS_PER_METER
    py = ORIGIN_PX[1] - y * PIXELS_PER_METER
    return int(px), int(py)


def px_to_world(px, py):
    x = (px - ORIGIN_PX[0]) / PIXELS_PER_METER
    y = -(py - ORIGIN_PX[1]) / PIXELS_PER_METER
    return x, y


# ----------------------------- LIDAR ray casting -----------------------------
def ray_segment_intersection(p0, d, a, b):
    """Return distance t along ray p0 + t*d to segment [a,b], or inf if none."""
    v1 = p0 - a
    v2 = b - a
    v3 = np.array([-d[1], d[0]])
    denom = np.dot(v2, v3)
    if abs(denom) < 1e-9:
        return np.inf
    t1 = (v2[0] * v1[1] - v2[1] * v1[0]) / denom  # 2D cross product (v2 x v1)
    t2 = np.dot(v1, v3) / denom
    if t1 >= 0.0 and 0.0 <= t2 <= 1.0:
        return t1
    return np.inf


def cast_rays(p0, angles, walls):
    """Vectorized version of ray_segment_intersection over every beam x wall.
    Returns the true range per beam, clipped at MAX_RANGE."""
    seg = np.asarray(walls, dtype=float)            # (M, 2, 2)
    a, b = seg[:, 0], seg[:, 1]                       # (M, 2)
    v1 = p0 - a                                       # (M, 2)
    v2 = b - a                                        # (M, 2)
    v3x = -np.sin(angles)[:, None]                    # (N, 1)  v3 = (-d_y, d_x)
    v3y = np.cos(angles)[:, None]
    denom = v2[:, 0] * v3x + v2[:, 1] * v3y           # (N, M)
    ok = np.abs(denom) >= 1e-9
    safe = np.where(ok, denom, 1.0)
    t1 = (v2[:, 0] * v1[:, 1] - v2[:, 1] * v1[:, 0]) / safe
    t2 = (v1[:, 0] * v3x + v1[:, 1] * v3y) / safe
    valid = ok & (t1 >= 0.0) & (t2 >= 0.0) & (t2 <= 1.0)
    t = np.where(valid, t1, np.inf)
    return np.minimum(t.min(axis=1), MAX_RANGE)


def collides(x, y, obstacles, margin=SENSOR_RADIUS):
    """True if point (x, y), grown by `margin`, overlaps any square obstacle."""
    lim = OBSTACLE_HALF + margin
    return any(abs(x - cx) < lim and abs(y - cy) < lim for cx, cy in obstacles)


def make_box(cx, cy, half=OBSTACLE_HALF):
    return [
        ((cx - half, cy - half), (cx + half, cy - half)),
        ((cx + half, cy - half), (cx + half, cy + half)),
        ((cx + half, cy + half), (cx - half, cy + half)),
        ((cx - half, cy + half), (cx - half, cy - half)),
    ]


def room_walls(half=WORLD_SIZE / 2):
    return [
        ((-half, -half), (half, -half)),
        ((half, -half), (half, half)),
        ((half, half), (-half, half)),
        ((-half, half), (-half, -half)),
    ]


def scan(sensor_pose, walls, rng, fov_deg, noise_sigma, dropout_p):
    """Physics-based ray-casting + probabilistic noise/dropout model.
    fov_deg, noise_sigma, dropout_p are passed in live so the interactive
    app can adjust the sensor's operating conditions (Lambda) at runtime."""
    x_s, y_s, theta_s = sensor_pose
    half_fov = math.radians(fov_deg) / 2.0
    if fov_deg >= 359.9:
        # full 360 deg: avoid duplicating the seam beam at +/-180
        local_angles = np.linspace(-half_fov, half_fov, N_BEAMS, endpoint=False)
    else:
        local_angles = np.linspace(-half_fov, half_fov, N_BEAMS)
    angles = theta_s + local_angles

    true_r = cast_rays(np.array([x_s, y_s]), angles, walls)

    # Gaussian noise corrupts only real returns; a beam that hit nothing stays
    # at r_max instead of turning into a phantom target just short of it.
    hit = true_r < MAX_RANGE
    noise = rng.normal(0.0, noise_sigma, size=len(angles))
    ranges = true_r.copy()
    ranges[hit] += noise[hit]
    ranges = np.clip(ranges, 0.0, MAX_RANGE)

    if dropout_p > 0:
        drop_mask = rng.random(len(angles)) < dropout_p
        ranges[drop_mask] = MAX_RANGE

    return angles, ranges


# ----------------------------- Main app -----------------------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Live LIDAR Simulator — lidarrun.py")
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("consolas", 16)
    font_small = pygame.font.SysFont("consolas", 13)

    rng = np.random.default_rng()

    sensor_x, sensor_y, sensor_theta = 0.0, 0.0, 0.0
    obstacles = []  # list of (cx, cy) centers
    fov_idx = FOV_OPTIONS.index(FOV_DEG_DEFAULT)
    noise_sigma = NOISE_SIGMA_DEFAULT
    dropout_p = DROPOUT_DEFAULT
    min_range_history = []  # rolling trend of min range, for the 2nd live graph

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0

        # ---------------- events ----------------
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_r:
                    obstacles.clear()
                elif event.key == pygame.K_f:
                    fov_idx = (fov_idx + 1) % len(FOV_OPTIONS)
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    noise_sigma = min(0.5, noise_sigma + 0.02)
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    noise_sigma = max(0.0, noise_sigma - 0.02)
                elif event.key == pygame.K_RIGHTBRACKET:
                    dropout_p = min(0.9, dropout_p + 0.05)
                elif event.key == pygame.K_LEFTBRACKET:
                    dropout_p = max(0.0, dropout_p - 0.05)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                if mx < ROOM_PANEL_W:  # click inside room panel only
                    wx, wy = px_to_world(mx, my)
                    if event.button == 1:  # left click = place
                        inside_room = (abs(wx) < WORLD_SIZE/2 - 0.4 and abs(wy) < WORLD_SIZE/2 - 0.4)
                        # don't drop a box on top of the sensor
                        on_sensor = collides(sensor_x, sensor_y, [(wx, wy)])
                        if inside_room and not on_sensor:
                            obstacles.append((wx, wy))
                    elif event.button == 3:  # right click = remove nearest
                        if obstacles:
                            dists = [ (wx-ox)**2 + (wy-oy)**2 for ox, oy in obstacles ]
                            nearest = int(np.argmin(dists))
                            if dists[nearest] < 1.0:  # within ~1m
                                obstacles.pop(nearest)

        # ---------------- keyboard: move / rotate sensor ----------------
        keys = pygame.key.get_pressed()
        move_speed = 3.0 * dt   # m/s
        rot_speed = 2.2 * dt    # rad/s
        dx = (keys[pygame.K_RIGHT] - keys[pygame.K_LEFT]) * move_speed
        dy = (keys[pygame.K_UP] - keys[pygame.K_DOWN]) * move_speed
        lim = WORLD_SIZE/2 - 0.2
        # move each axis separately so the sensor slides along obstacle faces
        nx = float(np.clip(sensor_x + dx, -lim, lim))
        if not collides(nx, sensor_y, obstacles):
            sensor_x = nx
        ny = float(np.clip(sensor_y + dy, -lim, lim))
        if not collides(sensor_x, ny, obstacles):
            sensor_y = ny
        if keys[pygame.K_a]:
            sensor_theta += rot_speed
        if keys[pygame.K_d]:
            sensor_theta -= rot_speed
        sensor_theta = (sensor_theta + math.pi) % (2 * math.pi) - math.pi  # keep in [-pi, pi)

        # ---------------- build world + scan ----------------
        obstacle_segs = [seg for (cx, cy) in obstacles for seg in make_box(cx, cy)]
        walls = room_walls() + obstacle_segs

        sensor_pose = (sensor_x, sensor_y, sensor_theta)
        fov_deg = FOV_OPTIONS[fov_idx]
        angles, ranges = scan(sensor_pose, walls, rng, fov_deg, noise_sigma, dropout_p)

        min_range_history.append(float(ranges.min()))
        if len(min_range_history) > HISTORY_LEN:
            min_range_history.pop(0)

        # ---------------- draw ----------------
        screen.fill(BLACK)

        # room panel background
        pygame.draw.rect(screen, (18, 18, 24), (0, 0, ROOM_PANEL_W, WIN_H))

        # walls
        for (a, b) in room_walls():
            pygame.draw.line(screen, WHITE, world_to_px(*a), world_to_px(*b), 3)

        # obstacles
        for (a, b) in obstacle_segs:
            pygame.draw.line(screen, GREEN, world_to_px(*a), world_to_px(*b), 3)

        # beams (dim brown) + hit points
        sx_px, sy_px = world_to_px(sensor_x, sensor_y)
        for i in range(N_BEAMS):
            hx = sensor_x + ranges[i] * math.cos(angles[i])
            hy = sensor_y + ranges[i] * math.sin(angles[i])
            hx_px, hy_px = world_to_px(hx, hy)
            pygame.draw.line(screen, (90, 60, 20), (sx_px, sy_px), (hx_px, hy_px), 1)
            pygame.draw.circle(screen, RED, (hx_px, hy_px), 2)

        # sensor marker + heading arrow
        pygame.draw.circle(screen, BLUE, (sx_px, sy_px), 7)
        hx = sensor_x + 0.5 * math.cos(sensor_theta)
        hy = sensor_y + 0.5 * math.sin(sensor_theta)
        pygame.draw.line(screen, BLUE, (sx_px, sy_px), world_to_px(hx, hy), 3)

        # divider
        pygame.draw.line(screen, GRAY, (ROOM_PANEL_W, 0), (ROOM_PANEL_W, WIN_H), 2)

        # ---------------- right panel: three live graphs ----------------
        pygame.draw.rect(screen, PANEL_BG, (ROOM_PANEL_W, 0, PLOT_PANEL_W, WIN_H))
        plot_x0 = ROOM_PANEL_W + 45
        plot_w = PLOT_PANEL_W - 80

        # --- Graph 1: range vs. beam angle (shows sensor-state / FOV dependency) ---
        g1_y0, g1_h = 45, 165
        title = font.render(f"1) Range vs. Beam Angle   (FOV={fov_deg:.0f} deg)", True, WHITE)
        screen.blit(title, (plot_x0, g1_y0 - 22))
        pygame.draw.line(screen, GRAY, (plot_x0, g1_y0), (plot_x0, g1_y0 + g1_h), 1)
        pygame.draw.line(screen, GRAY, (plot_x0, g1_y0 + g1_h), (plot_x0 + plot_w, g1_y0 + g1_h), 1)
        local_angles_deg = np.degrees(angles - sensor_theta)
        # wrap into [-180,180] so 360-deg mode doesn't draw a stray diagonal
        local_angles_deg = (local_angles_deg + 180) % 360 - 180
        order = np.argsort(local_angles_deg)
        pts = []
        for i in order:
            gx = plot_x0 + (local_angles_deg[i] + fov_deg / 2) / fov_deg * plot_w
            gy = g1_y0 + g1_h - (ranges[i] / MAX_RANGE) * g1_h
            pts.append((gx, gy))
        if len(pts) > 1:
            pygame.draw.lines(screen, GREEN, False, pts, 2)
        screen.blit(font_small.render("0", True, GRAY), (plot_x0 - 18, g1_y0 + g1_h - 8))
        screen.blit(font_small.render(f"{MAX_RANGE:.0f}m", True, GRAY), (plot_x0 - 30, g1_y0 - 6))
        screen.blit(font_small.render("beam angle (relative to heading)", True, GRAY), (plot_x0, g1_y0 + g1_h + 6))

        # --- Graph 2: min range over time (shows target/sensor-motion dependency live) ---
        g2_y0, g2_h = g1_y0 + g1_h + 55, 130
        title2 = font.render("2) Min Range Over Time (trend)", True, WHITE)
        screen.blit(title2, (plot_x0, g2_y0 - 22))
        pygame.draw.line(screen, GRAY, (plot_x0, g2_y0), (plot_x0, g2_y0 + g2_h), 1)
        pygame.draw.line(screen, GRAY, (plot_x0, g2_y0 + g2_h), (plot_x0 + plot_w, g2_y0 + g2_h), 1)
        if len(min_range_history) > 1:
            pts2 = []
            for i, mr in enumerate(min_range_history):
                gx = plot_x0 + i / (HISTORY_LEN - 1) * plot_w
                gy = g2_y0 + g2_h - (mr / MAX_RANGE) * g2_h
                pts2.append((gx, gy))
            pygame.draw.lines(screen, (255, 170, 60), False, pts2, 2)
        screen.blit(font_small.render("0", True, GRAY), (plot_x0 - 18, g2_y0 + g2_h - 8))
        screen.blit(font_small.render(f"{MAX_RANGE:.0f}m", True, GRAY), (plot_x0 - 30, g2_y0 - 6))
        screen.blit(font_small.render(f"time  (last {HISTORY_LEN} frames)", True, GRAY), (plot_x0, g2_y0 + g2_h + 6))

        # --- Graph 3: histogram of current ranges (shows noise/dropout / environment dependency) ---
        g3_y0, g3_h = g2_y0 + g2_h + 55, 110
        title3 = font.render(f"3) Range Histogram   (sigma={noise_sigma:.2f}m, drop={dropout_p:.2f})", True, WHITE)
        screen.blit(title3, (plot_x0, g3_y0 - 22))
        pygame.draw.line(screen, GRAY, (plot_x0, g3_y0), (plot_x0, g3_y0 + g3_h), 1)
        pygame.draw.line(screen, GRAY, (plot_x0, g3_y0 + g3_h), (plot_x0 + plot_w, g3_y0 + g3_h), 1)
        n_bins = 24
        counts, edges = np.histogram(ranges, bins=n_bins, range=(0, MAX_RANGE))
        max_count = max(counts.max(), 1)
        bin_w = plot_w / n_bins
        for i, c in enumerate(counts):
            bh = (c / max_count) * g3_h
            bx = plot_x0 + i * bin_w
            by = g3_y0 + g3_h - bh
            pygame.draw.rect(screen, (100, 160, 255), (bx + 1, by, max(bin_w - 2, 1), bh))
        screen.blit(font_small.render("0", True, GRAY), (plot_x0 - 4, g3_y0 + g3_h + 6))
        screen.blit(font_small.render(f"{MAX_RANGE:.0f}m", True, GRAY), (plot_x0 + plot_w - 20, g3_y0 + g3_h + 6))

        # --- live readouts ---
        info_y = g3_y0 + g3_h + 35
        lines = [
            f"Sensor pos : ({sensor_x:+.2f}, {sensor_y:+.2f}) m   Heading: {math.degrees(sensor_theta):+.1f} deg",
            f"Obstacles  : {len(obstacles)}    FOV: {fov_deg:.0f} deg    FPS: {clock.get_fps():.0f}",
            f"Min/Max/Mean range : {ranges.min():.2f} / {ranges.max():.2f} / {ranges.mean():.2f} m",
        ]
        for i, line in enumerate(lines):
            surf = font_small.render(line, True, WHITE)
            screen.blit(surf, (plot_x0, info_y + i * 20))

        controls = [
            "Controls: LClick=add box  RClick=remove  Arrows=move",
            "  A/D=rotate  F=cycle FOV  +/-=noise sigma",
            "  [ ]=dropout  R=clear  Esc=quit",
        ]
        for i, line in enumerate(controls):
            surf = font_small.render(line, True, GRAY)
            screen.blit(surf, (plot_x0, info_y + 70 + i * 18))

        pygame.display.flip()

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
