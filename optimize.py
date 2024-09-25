import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize

def random_color():
	return np.random.rand(3)  # Generates an array of 3 random numbers between 0 and 1

def simple_icp(src,tgt):
	src.estimate_normals()
	tgt.estimate_normals()
	source = src.voxel_down_sample(0.001)
	target = tgt.voxel_down_sample(0.001)
	initial_trans = np.identity(4)
	result_icp = o3d.pipelines.registration.registration_icp(
		source, target, 0.02, initial_trans,
		o3d.pipelines.registration.TransformationEstimationPointToPlane())
	return result_icp.transformation

def evaluate_trans(trans):
	t_vec = trans[:3,3]
	R_trans = np.copy(trans[:3,:3])
	rot =  Rotation.from_matrix(R_trans)
	quat = rot.as_quat()
	
	t = np.linalg.norm(t_vec,ord=2)
	th = 2*np.arccos(quat[3])
	print(t,th)
	return t,th


def draw_pcd(params):
	score = 0
	pcd = o3d.geometry.PointCloud()
	cnt = 0
	last_pcd = None
	for id in range(3,5):
		#dir0 = f'/Users/kisho/open3d/Experiments/dataset_paramEst_auto4/data{id}'
		dir0 = f'/Users/kisho/open3d/Experiments/data_pcd/data{id}'
		# 点群データの読み込み
		T1 = np.load(f"{dir0}/trans1.npy")
		T2 = np.load(f"{dir0}/trans2.npy")
		T3 = np.load(f"{dir0}/trans3.npy")
		#T4 = np.load(f"{dir0}/trans4.npy")
		T2[0,3] = params[0]
		T2[1,3] = params[1]
		T2[2,3] = params[2]
		T=T1@T2@T3

		new_pcd = o3d.io.read_point_cloud(f"{dir0}/new_pcd.ply")
		new_pcd = new_pcd.voxel_down_sample(0.001)
		color = random_color()  # Can use a fixed colormap or random colors
		new_pcd.paint_uniform_color(color)
		new_pcd.transform(T)
		if cnt != 0:
			trans = simple_icp(new_pcd, pcd)
			#new_pcd.transform(trans)
			#print(trans)
			try:
				t,th = evaluate_trans(trans)
			except:
				t=100

			print(params,t)
			score += t

		#new_pcd.transform(np.linalg.inv(T4))
		pcd += new_pcd
		cnt+=1
	o3d.visualization.draw_geometries([pcd])
	return score

"""
initial_params = np.array([0.0, 0.0, 0.002])
result = minimize(draw_pcd, initial_params)

# 4. 結果の表示
print(f"最適化された値: {result.x}")
print(f"評価関数の最小値: {result.fun}")
"""

#for d in np.arange(-0.1,0.1,0.01):
#	draw_pcd(d)
draw_pcd([0.016,0.0,0.019])