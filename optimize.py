import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize

np.random.seed(1)

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

def params2matrix(params):
	rot = Rotation.from_euler('xyz', [0.0, -np.pi/2, np.pi/2])
	quaternion = rot.as_quat()
	#quaternion = [0.5,-0.5,0.5,0.5]
	rotation_matrix = Rotation.from_quat(quaternion).as_matrix()
	translation_vector = np.array([params[0], params[1], params[2]])
	transformation_matrix = np.eye(4)
	transformation_matrix[:3, :3] = rotation_matrix  # 回転行列を代入
	transformation_matrix[:3, 3] = translation_vector  # 平行移動ベクトルを代入
	return transformation_matrix


def draw_pcd(params):
	score = 0
	pcd = o3d.geometry.PointCloud()
	cnt = 0
	last_pcd = None
	for id in range(7):
		dir0 = f'/Users/kisho/open3d/Experiments/data_pcd/data{id}'
		T1 = np.load(f"{dir0}/trans1.npy")
		T2 = np.load(f"{dir0}/trans2.npy")
		T3 = np.load(f"{dir0}/trans3.npy")
		T2 = params2matrix(params)
		T=T1@T2@T3

		new_pcd = o3d.io.read_point_cloud(f"{dir0}/new_pcd.ply")
		new_pcd = new_pcd.voxel_down_sample(0.001)
		color = random_color()  # Can use a fixed colormap or random colors
		new_pcd.paint_uniform_color(color)
		new_pcd.transform(T)
		if cnt != 0:
			trans = simple_icp(new_pcd, pcd)
			try:
				t,th = evaluate_trans(trans)
			except:
				t=100

			print(params,t)
			score += t
			new_pcd.transform(trans)

		pcd += new_pcd
		cnt+=1
	o3d.visualization.draw_geometries([pcd])
	return score

#initial_params = np.array([0.0, 0.0, 0.002])
#result = minimize(draw_pcd, initial_params)

# 4. 結果の表示
#print(f"最適化された値: {result.x}")
#print(f"評価関数の最小値: {result.fun}")

draw_pcd([0.0,0.0,0.0])