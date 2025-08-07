#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import open3d as o3d
import std_msgs.msg

# ---------- USER PARAMETERS ----------
initial_center = np.array([5.368, 0.0, 0.5])  # initial bounding box center (x, y, z)
box_size = np.array([1.0, 1.0, 1.0])          # length, width, height of the box
# -------------------------------------

prev_pcd = None
publisher = None
box_center = initial_center.copy()

def compute_normals(pcd, radius=0.5, max_nn=30):
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=max_nn))
    return np.asarray(pcd.normals)

def angular_difference(n1, n2):
    cos_theta = np.clip(np.sum(n1 * n2, axis=1), -1.0, 1.0)
    return np.arccos(cos_theta)

def identify_dynamic_points(pcd_t1, pcd_t2, threshold_deg=20):
    normals_t1 = compute_normals(pcd_t1)
    normals_t2 = compute_normals(pcd_t2)
    angle_diff = angular_difference(normals_t1, normals_t2)
    threshold_rad = np.deg2rad(threshold_deg)
    return angle_diff > threshold_rad

def get_points_in_box(points, center, size):
    lower = center - size / 2
    upper = center + size / 2
    mask = np.all((points >= lower) & (points <= upper), axis=1)
    return mask

def colorize_pointcloud(points, dynamic_mask, in_box_mask):
    colors = np.tile([0.3, 0.3, 0.3], (len(points), 1))           # gray for all points
    colors[in_box_mask] = [0.5, 0.5, 0.5]                         # light gray for points inside the box
    colors[in_box_mask & dynamic_mask] = [1.0, 0.0, 0.0]          # red for dynamic points inside the box
    return colors

def ros_to_open3d(pcl_msg):
    points = np.array(list(pc2.read_points(pcl_msg, skip_nans=True, field_names=("x", "y", "z"))))
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd, points

def open3d_to_ros(points, colors, frame_id="velodyne"):
    rgb = (colors * 255).astype(np.uint8)
    rgb_packed = (rgb[:, 0].astype(np.uint32) << 16) | (rgb[:, 1].astype(np.uint32) << 8) | rgb[:, 2].astype(np.uint32)
    cloud_data = list(zip(points[:, 0], points[:, 1], points[:, 2], rgb_packed))

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgb', 12, PointField.UINT32, 1),
    ]

    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    return pc2.create_cloud(header, fields, cloud_data)

def update_box_center(points, dynamic_mask, in_box_mask):
    global box_center
    relevant = points[in_box_mask & dynamic_mask]
    if len(relevant) > 0:
        new_center = np.mean(relevant, axis=0)
        box_center[:] = new_center

def callback(msg):
    global prev_pcd, publisher, box_center
    curr_pcd, curr_points = ros_to_open3d(msg)

    if prev_pcd is not None and len(curr_points) == len(prev_pcd.points):
        prev_points = np.asarray(prev_pcd.points)
        prev_pcd.points = o3d.utility.Vector3dVector(prev_points)

        mask_box = get_points_in_box(curr_points, box_center, box_size)

        pcd_prev_box = prev_pcd.select_by_index(np.where(mask_box)[0])
        pcd_curr_box = curr_pcd.select_by_index(np.where(mask_box)[0])

        if len(pcd_prev_box.points) > 0 and len(pcd_curr_box.points) > 0:
            dynamic_mask_box = identify_dynamic_points(pcd_prev_box, pcd_curr_box)
            full_dynamic_mask = np.zeros(len(curr_points), dtype=bool)
            full_dynamic_mask[np.where(mask_box)[0]] = dynamic_mask_box

            update_box_center(curr_points, full_dynamic_mask, mask_box)
            colors = colorize_pointcloud(curr_points, full_dynamic_mask, mask_box)
            ros_msg = open3d_to_ros(curr_points, colors)
            publisher.publish(ros_msg)

    prev_pcd = curr_pcd

if __name__ == "__main__":
    rospy.init_node("dynamic_detector_node")
    publisher = rospy.Publisher("/dynamic_points", PointCloud2, queue_size=1)
    rospy.Subscriber("/velodyne_points", PointCloud2, callback, queue_size=1)
    rospy.loginfo("Tracking dynamic object in bounding box...")
    rospy.spin()
