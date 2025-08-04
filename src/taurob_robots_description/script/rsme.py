#!/usr/bin/env python3

import rosbag
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt
from nav_msgs.msg import OccupancyGrid

# ==== PARAMETERS ====
bag_unfiltered = "mapa_sem_filtro.bag"
bag_filtered = "mapa_sem_parametros_filtro.bag"
map_topic = "/map"

theta_deg = 110.9
dx = 0.55
dy = 0.0
dilation_kernel_size = 3

# ==== FUNCTIONS ====

def load_final_map_from_bag(bag_file, topic):
    last_msg = None
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic_name, msg, t in bag.read_messages(topics=[topic]):
            last_msg = msg
    return last_msg

def occupancy_grid_to_numpy(occ_grid):
    data = np.array(occ_grid.data).reshape((occ_grid.info.height, occ_grid.info.width))
    binary_map = (data > 50).astype(np.uint8)
    return binary_map, occ_grid.info

def transform_map(map_img, info, theta_deg, dx, dy):
    h, w = map_img.shape
    center = (w // 2, h // 2)

    rot_mat = cv2.getRotationMatrix2D(center, theta_deg, 1.0)
    rotated = cv2.warpAffine(map_img, rot_mat, (w, h), flags=cv2.INTER_NEAREST)

    dx_px = int(dx / info.resolution)
    dy_px = int(dy / info.resolution)
    trans_mat = np.float32([[1, 0, dx_px], [0, 1, -dy_px]])
    transformed = cv2.warpAffine(rotated, trans_mat, (w, h), flags=cv2.INTER_NEAREST)

    return transformed

def create_ground_truth_map(info, walls):
    map_img = np.zeros((info.height, info.width), dtype=np.uint8)
    origin_x = info.origin.position.x
    origin_y = info.origin.position.y

    for x, y, w, h in walls:
        px = int((x - origin_x) / info.resolution)
        py = int((y - origin_y) / info.resolution)
        pw = int(w / info.resolution)
        ph = int(h / info.resolution)
        cv2.rectangle(map_img, (px - pw//2, py - ph//2), (px + pw//2, py + ph//2), 1, -1)

    return map_img

def compute_rmse(map1, map2):
    return np.sqrt(np.mean((map1 - map2) ** 2))

def compute_iou(map1, map2):
    intersection = np.logical_and(map1, map2).sum()
    union = np.logical_or(map1, map2).sum()
    return intersection / union if union != 0 else 0

def compute_distance_weighted_rmse(pred_map, gt_map):
    gt_inverted = 1 - gt_map
    distance_map = distance_transform_edt(gt_inverted)
    distance_map = distance_map / (np.max(distance_map) + 1e-6)

    error = (pred_map - gt_map) ** 2
    weighted_error = error * distance_map
    return np.sqrt(np.mean(weighted_error)), distance_map

def downsample_map(m, factor=10):
    return m[::factor, ::factor]

# ==== MAIN ====
if __name__ == "__main__":
    msg_unfiltered = load_final_map_from_bag(bag_unfiltered, map_topic)
    msg_filtered = load_final_map_from_bag(bag_filtered, map_topic)

    if msg_unfiltered is None or msg_filtered is None:
        print("❌ Could not find the /map topic in one or both bags.")
        exit()

    map_unfiltered, info = occupancy_grid_to_numpy(msg_unfiltered)
    map_filtered, _ = occupancy_grid_to_numpy(msg_filtered)

    map_unfiltered = transform_map(map_unfiltered, info, theta_deg, dx, dy)
    map_filtered = transform_map(map_filtered, info, theta_deg, dx, dy)

    # DILATE predicted maps to match GT wall thickness
    kernel = np.ones((dilation_kernel_size, dilation_kernel_size), np.uint8)
    map_unfiltered_dil = cv2.dilate(map_unfiltered, kernel, iterations=1)
    map_filtered_dil = cv2.dilate(map_filtered, kernel, iterations=1)

    # Ground truth map (walls)
    walls = [
        (0, 10, 20, 0.2),
        (0, -10, 20, 0.2),
        (10, 0, 0.2, 20),
        (-10, 0, 0.2, 20),
    ]
    gt_map = create_ground_truth_map(info, walls)

    # Metrics
    rmse_unf = compute_rmse(map_unfiltered_dil, gt_map)
    iou_unf = compute_iou(map_unfiltered_dil, gt_map)
    w_rmse_unf, dist_map = compute_distance_weighted_rmse(map_unfiltered_dil, gt_map)

    rmse_flt = compute_rmse(map_filtered_dil, gt_map)
    iou_flt = compute_iou(map_filtered_dil, gt_map)
    w_rmse_flt, _ = compute_distance_weighted_rmse(map_filtered_dil, gt_map)

    print("📊 Unfiltered Map (dilated):")
    print(f"   RMSE           : {rmse_unf:.4f}")
    print(f"   IoU            : {iou_unf:.4f}")
    print(f"   Weighted RMSE  : {w_rmse_unf:.4f}")

    print("📊 Filtered Map (dilated):")
    print(f"   RMSE           : {rmse_flt:.4f}")
    print(f"   IoU            : {iou_flt:.4f}")
    print(f"   Weighted RMSE  : {w_rmse_flt:.4f}")

    # Downsample maps para plot mais leve
    map_unfiltered_dil_ds = downsample_map(map_unfiltered_dil)
    map_filtered_dil_ds = downsample_map(map_filtered_dil)
    gt_map_ds = downsample_map(gt_map)
    dist_map_ds = downsample_map(dist_map)

    # === Visualization ===
    fig, axes = plt.subplots(1, 4, figsize=(24, 6))  # Tamanho maior para visibilidade no Overleaf

    titles = [
        "Unfiltered Map (Dilated)",
        "Filtered Map (Dilated)",
        "Ground Truth Map",
        "Distance Map"
    ]

    maps = [
        map_unfiltered_dil_ds,
        map_filtered_dil_ds,
        gt_map_ds,
        dist_map_ds
    ]

    for ax, title, m in zip(axes, titles, maps):
        ax.set_facecolor('black')  # Fundo de cada subplot
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.axis('off')

        if title != "Distance Map":
            ny, nx = m.shape
            x = np.arange(nx + 1)
            y = np.arange(ny + 1)
            ax.pcolormesh(x, y, m, shading='auto', cmap='gray_r')
        else:
            levels = np.linspace(np.min(m), np.max(m), 10)
            ax.contourf(m, levels=levels, cmap='hot')

    plt.tight_layout(pad=1.0, w_pad=2.0)  # Ajusta espaçamento entre as imagens
    plt.savefig('rmse.png', dpi=300)  # Salva em PGF com downsample para evitar lentidão
