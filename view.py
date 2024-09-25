import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize


dir0 = '/Users/kisho/open3d/Experiments/data'
# データの読み込み
#data = np.load('/home/realsense/kisho_ws/ubicomp/3d_reconstruction/src/calibration_tool/calibration_tool/info/savedata.npy')

def get_std(points):
    # 各軸ごとの分散
    var_x = np.var(points[:, 0])
    var_y = np.var(points[:, 1])
    var_z = np.var(points[:, 2])
    std_x = np.std(points[:, 0])
    std_y = np.std(points[:, 1])
    std_z = np.std(points[:, 2])
    return (std_x+std_y+std_z)/3.0

def calc_diff(data):
    l = len(data)
    score = 0.0
    for i in range(l-1):
        diff = data[i+1]-data[i]
        score += np.linalg.norm(diff,ord=2)
        #print(score)
    return score/(l-1)


c6_camera_list = []
c6_map_list = []
c6_map_calc_list = []
T1_list = []
T2_list = []
T3_list = []


# 指定された数字
all_numbers = list(range(5))
# パスするリスト
pass_list = []

# pass_listを含まないリストを作成
filtered_list = [num for num in all_numbers if num not in pass_list]

for i in filtered_list:
    if  i in  pass_list:
        continue

    T1 = np.load(f"{dir0}/T1_{i}.npy")
    T1_list.append(T1)
    T2 = np.load(f"{dir0}/T2_{i}.npy")
    T2[0,3] = -0.02892
    T2[1,3] = 0.005472 #0.1037
    T2[2,3] = 0.03032 #0.02601 #-0.03360
    #print(T2)
    T2_list.append(T2)
    T3 = np.load(f"{dir0}/T3_{i}.npy")
    T3_list.append(T3)
    c6_camera = np.load(f"{dir0}/c6_camera_{i}.npy")
    c6_camera_list.append(c6_camera)
    c6_map = np.load(f"{dir0}/c6_map_{i}.npy")
    c6_map_list.append(c6_map)
    c6_map_calc = T1@T2@T3@np.array([c6_camera[0],c6_camera[1],c6_camera[2],1])
    c6_map_calc_list.append(c6_map_calc[:3])


def evaluate_function(params):
    temp = []
    for i in range(len(T1_list)):
      T1 = T1_list[i]
      T2 = T2_list[i]
      T2[0,3] = params[0]
      T2[1,3] = params[1]
      T2[2,3] = params[2]
      #print(T2)
      T3 = T3_list[i]
      c6_camera = c6_camera_list[i]
      c6_map_calc = T1@T2@T3@np.array([c6_camera[0],c6_camera[1],c6_camera[2],1])
      temp.append(c6_map_calc[:3])
    temp = np.array(temp)
    #score = calc_diff(temp)
    score = get_std(temp)
    return score

initial_params = np.array([0.0, 0.0, 0.0])
data = np.array(c6_map_calc_list)

score = evaluate_function([0.0,0.0,0.0])
print(score)
result = minimize(evaluate_function, initial_params)

# 4. 結果の表示
print(f"最適化された値: {result.x}")
print(f"評価関数の最小値: {result.fun}")
#score = evaluate([0.12,-0.057])
#print(score)


#print(data[1]-data[0])
#print(data[2]-data[1])
#print(data[3]-data[2])
#print(data[4]-data[3])

# x, y, z座標に分ける（データの形状によっては変更が必要）
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]

x_median = np.median(x)
y_median = np.median(y)
z_median = np.median(z)

# 3Dグラフの作成
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 点をプロット
ax.scatter(x, y, z)
# 各点にIDを表示
for i in range(len(x)):
    ax.text(x[i], y[i], z[i], str(filtered_list[i]), color='red')

# ラベルの設定
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
# 各軸の表示範囲を中央値±0.005に設定
d = 0.01
ax.set_xlim([x_median - d, x_median + d])
ax.set_ylim([y_median - d, y_median + d])
ax.set_zlim([z_median - d, z_median + d])


# グラフの表示
plt.show()

