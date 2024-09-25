import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation

def random_color():
    return np.random.rand(3)  


def draw_pcd(d):
	print(d)
	pcd = o3d.geometry.PointCloud()
	last_pcd = None
	for id in range(7):
		dir0 = f'/Users/kisho/open3d/Experiments/data_pcd/data{id}'
		T1 = np.load(f"{dir0}/trans1.npy")
		T2 = np.load(f"{dir0}/trans2.npy")
		T3 = np.load(f"{dir0}/trans3.npy")
		T2[0,3] = 0.0
		T2[1,3] = -0.005
		T2[2,3] = 0.0225
		T=T1@T2@T3

		new_pcd = o3d.io.read_point_cloud(f"{dir0}/new_pcd.ply")
		new_pcd = new_pcd.voxel_down_sample(0.001)
		color = random_color()  # Can use a fixed colormap or random colors
		new_pcd.paint_uniform_color(color)
		new_pcd.transform(T)
		pcd += new_pcd
	o3d.visualization.draw_geometries([pcd])


draw_pcd(0.018)