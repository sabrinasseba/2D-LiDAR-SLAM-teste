#!/usr/bin/env python3
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np
from sklearn.cluster import DBSCAN
import open3d as o3d
import CSF
import pandas as pd

class BoxFilter:
    def __init__(self):
        rospy.init_node('box_filter_node', anonymous=True)

        # Tópico do LiDAR Velodyne
        lidar_topic = "/velodyne_points"

        # Subscrição ao tópico do LiDAR
        self.lidar_sub = rospy.Subscriber(lidar_topic, PointCloud2, self.pointcloud_callback)

        # Publicação dos pontos filtrados
        self.filtered_pub = rospy.Publisher("/filtered_box_points", PointCloud2, queue_size=1)

        rospy.loginfo("Box Filter Node is running! Filtering ground points and isolating the moving box.")

    def pointcloud_callback(self, msg):
        """ Processa a nuvem de pontos do LiDAR e filtra automaticamente os pontos da caixa, removendo o chão. """
        points = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)))

        if len(points) == 0:
            rospy.logwarn("No valid LiDAR points detected.")
            return

        # Remover pontos muito distantes (feixes infinitos)
        max_distance = 12.0  # Ajuste conforme necessário
        valid_points = points[np.linalg.norm(points[:, :2], axis=1) < max_distance]

        if len(valid_points) == 0:
            rospy.logwarn("No valid collision points detected.")
            return

        # Convertendo para DataFrame para aplicar filtragem de solo via CSF
        df = pd.DataFrame(valid_points, columns=['x', 'y', 'z'])
        df_filtered = self.ground_filtering(df)  # Aplica a filtragem do solo

        if len(df_filtered) == 0:
            rospy.logwarn("All detected points were ground. No object detected.")
            return

        # Aplicar clustering para detectar a maior concentração de pontos (o quadrado)
        clustering = DBSCAN(eps=2.0, min_samples=2).fit(df_filtered[['x', 'y']])
        labels = clustering.labels_

        # Encontrar o cluster mais denso (assumindo que é o quadrado)
        unique_labels, counts = np.unique(labels, return_counts=True)
        if len(unique_labels) == 1 and unique_labels[0] == -1:
            rospy.logwarn("No clusters detected.")
            return

        # Seleciona o cluster mais populoso (o quadrado em movimento)
        box_cluster = unique_labels[np.argmax(counts)]

        # Filtra os pontos que pertencem ao cluster identificado
        filtered_points = df_filtered.to_numpy()[labels == box_cluster]

        if len(filtered_points) == 0:
            rospy.logwarn("No points detected for the moving box.")
            return

        # Criar e publicar nova mensagem PointCloud2 com os pontos da caixa
        filtered_msg = self.create_pointcloud2(msg.header, filtered_points)
        self.filtered_pub.publish(filtered_msg)

        rospy.loginfo("Published {} filtered points from the moving box.".format(len(filtered_points)))

    def ground_filtering(self, df):
        """ Filtragem de solo usando Cloth Simulation Filtering (CSF). """
        csf = CSF.CSF()
        csf.params.bSloopSmooth = False
        csf.params.cloth_resolution = 0.5  # Define a resolução da malha para separar o solo

        # Conversão para Open3D
        points = np.array(df[['x', 'y', 'z']])
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])

        # Aplicar CSF
        csf.setPointCloud(pcd.points)
        ground = CSF.VecInt()
        non_ground = CSF.VecInt()
        csf.do_filtering(ground, non_ground)

        # Retornar apenas os pontos não pertencentes ao solo
        df_filtered = pd.DataFrame(points[non_ground], columns=['x', 'y', 'z'])
        return df_filtered

    def create_pointcloud2(self, header, points):
        """ Cria uma mensagem PointCloud2 a partir de um array numpy. """
        fields = [
            pc2.PointField(name="x", offset=0, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name="y", offset=4, datatype=pc2.PointField.FLOAT32, count=1),
            pc2.PointField(name="z", offset=8, datatype=pc2.PointField.FLOAT32, count=1)
        ]
        return pc2.create_cloud(header, fields, points)

if __name__ == "__main__":
    try:
        BoxFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
