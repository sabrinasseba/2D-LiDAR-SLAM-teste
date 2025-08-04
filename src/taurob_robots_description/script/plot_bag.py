#!/usr/bin/env python3
import os
import math
import rosbag
import numpy as np
import matplotlib as mpl
mpl.use('pgf')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy import ndimage
from skimage import measure
from matplotlib.lines import Line2D

# ==== LaTeX Style with PGF ====
def set_size(width_pt, fraction=1, subplots=(1, 1)):
    fig_width_pt = width_pt * fraction
    inches_per_pt = 1 / 72.27
    golden_ratio = 0.8
    fig_width_in = fig_width_pt * inches_per_pt
    fig_height_in = fig_width_in * golden_ratio * (subplots[0] / subplots[1])
    return (fig_width_in, fig_height_in)

plt.style.use('ggplot')
plt.rc('text', usetex=True)
plt.rc('font', family='serif')
plt.rc('pgf', rcfonts=False)

os.makedirs('plots', exist_ok=True)

# ==== Read rosbag ====
bag = rosbag.Bag('mapa_semfiltro_uma_parada.bag')  # adjust path if needed

# === Get latest /map message ===
last_map_msg = None
for topic, msg, t in bag.read_messages(topics=['/map']):
    last_map_msg = msg
bag.close()

if last_map_msg is None:
    raise RuntimeError("No /map message found in the bag.")

# ==== World to Map Conversion ====
res = last_map_msg.info.resolution
origin_x = last_map_msg.info.origin.position.x
origin_y = last_map_msg.info.origin.position.y

def map_to_world(mx, my):
    x = origin_x + mx * res
    y = origin_y + my * res
    return x, y

def transform_point(x, y, theta_deg=0.0, dx=0.0, dy=0.0):
    """Apply 2D rotation and translation to a point (x, y)."""
    theta = math.radians(theta_deg)
    x_rot = math.cos(theta) * x - math.sin(theta) * y
    y_rot = math.sin(theta) * x + math.cos(theta) * y
    return x_rot + dx, y_rot + dy

# === Configurable Transform Parameters (for the map) ===
theta_deg = -111.6  # map rotation
dx = 0.6          # map translation in meters
dy = 0.0

# ==== Load and Prepare Map ====
width = last_map_msg.info.width
height = last_map_msg.info.height
data = np.array(last_map_msg.data).reshape((height, width))
data = np.flipud(data)

# ==== Occupied Cell Contours ====
binary_map = (data == 100).astype(np.uint8)
contours = measure.find_contours(binary_map, 0.5)

# ==== Plot ====
fig, ax = plt.subplots(figsize=set_size(516))

# --- Plot Map Contours (transformed) ---
for i, contour in enumerate(contours):
    x_world, y_world = [], []
    for y_map, x_map in contour:
        x, y = map_to_world(x_map, height - y_map)
        x_t, y_t = transform_point(x, y, theta_deg, dx, dy)
        x_world.append(x_t)
        y_world.append(y_t)
    if i == 0:
        ax.plot(x_world, y_world, linewidth=0.5, color='blue')
    else:
        ax.plot(x_world, y_world, linewidth=0.5, color='blue')

# === Obstacles from .world ===
#obstacles = [
#    {"pose": (-6, 3), "size": (1, 1), "color": 'red'},
#    {"pose": (5, -4), "size": (1, 1), "color": 'orange'},
#    {"pose": (-4, -4), "size": (0.1, 2.0), "color": 'green'}
#]

#for obs in obstacles:
#    x, y = obs["pose"]
#    w, h = obs["size"]
#    rect = patches.Rectangle(
#        (x - w / 2, y - h / 2), w, h,
#        linewidth=1,
#        edgecolor=obs["color"],
#        facecolor='none',
#        linestyle='-'
#    )
#    ax.add_patch(rect)

# === Finalize Plot ===
ax.set_aspect('equal')
ax.grid(True, linestyle='--', linewidth=0.5, color='gray', alpha=0.7)

ax.set_xlabel(r"Position $x$ [m]")
ax.set_ylabel(r"Position $y$ [m]")

#legend_elements = [
    #Line2D([0], [0], color='blue', lw=1, label='Map with filter and parameters'),
    #Line2D([0], [0], color='green', lw=1, label=r'$\mathcal{O}_1$'),
    #Line2D([0], [0], color='red', lw=1, label=r'$\mathcal{O}_2$'),
    #Line2D([0], [0], color='orange', lw=1, label=r'$\mathcal{O}_3$'),
#]

#ax.legend(
    #handles=legend_elements,
#    loc='lower right',
#    fontsize=8,
#    frameon=True,
#    facecolor='white',
#    edgecolor='black',
#    framealpha=1.0
#)

ax.set_xlim(-15, 15)
ax.set_ylim(-15, 15)

# === Save Figures ===
#plt.savefig(.pdf', format='pdf', dpi=300, bbox_inches='tight')
plt.savefig('sem_filtro.pgf', format='pgf', bbox_inches='tight', facecolor='white')