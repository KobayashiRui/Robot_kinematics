# direct kinematics
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def get_transform_matrix(rotation_matrix, position_vec):
    return np.array([[rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], position_vec[0]],
                     [rotation_matrix[1][0], rotation_matrix[1][1],
                         rotation_matrix[1][2], position_vec[1]],
                     [rotation_matrix[2][0], rotation_matrix[2][1],
                         rotation_matrix[2][2], position_vec[2]],
                     [0., 0., 0., 1.]])


class Link:
    # position_vec : , static_rotation :, end_effector :
    def __init__(self, position_vec, static_rotation_matrix, end_effector=False):
        self.position_vec = position_vec
        self.static_rotation_matrix = static_rotation_matrix
        self.end_effector = end_effector

    def get_link_transform_matrix(self, radian=None):
        if(not self.end_effector or not radian == None):
            dynamic_rotation_matrix = np.array([[np.cos(radian), -np.sin(radian), 0.],
                                                [np.sin(radian), np.cos(radian), 0.],
                                                [0., 0., 1.]])
            rotation_matrix = np.matmul(
                self.static_rotation_matrix, dynamic_rotation_matrix)
        else:
            rotation_matrix = self.static_rotation_matrix

        return get_transform_matrix(rotation_matrix, self.position_vec)


class Arm:
    def __init__(self, Link_Datas):
        self.Links = []
        before_joint_axis = 0  # init value is Roll = 0
        for link in Link_Datas:
            # get statick rotation matrix
            if(link[1] == 2 and before_joint_axis == 1):
                static_rotation_matrix = np.array([[1., 0., 0.],
                                                   [0., 0., -1.],
                                                   [0., 1., 0.]])
            elif(link[1] == 2 or link[1] == before_joint_axis):  # 回転軸が変わらないorEnd-effector
                static_rotation_matrix = np.eye(3)  # 4x4 単位行列
            elif(before_joint_axis == 0 and link[1] == 1):  # ロール→ピッチ
                static_rotation_matrix = np.array([[1., 0., 0.],
                                                   [0., 0., 1.],
                                                   [0., -1., 0.]])
            elif(before_joint_axis == 1 and link[1] == 0):  # ピッチ→ロール
                static_rotation_matrix = np.array([[1., 0., 0.],
                                                   [0., 0., -1.],
                                                   [0., 1., 0.]])
            # get position_vec
            if(before_joint_axis == 0):
                position_vec = link[0]
            else:
                position_vec = [link[0][0], -link[0][2], link[0][1]]

            if(link[1] == 2):
                self.Links.append(
                    Link(position_vec, static_rotation_matrix, True))
            else:
                self.Links.append(Link(position_vec, static_rotation_matrix))

            before_joint_axis = link[1]

    def Arm_Preview(self):
        x_data = [0]
        y_data = [0]
        z_data = [0]

        fig = plt.figure()
        ax = Axes3D(fig)
        ax.set_xlim(-25, 25)
        ax.set_ylim(-25, 25)
        ax.set_zlim(0, 50)

        result = self.transform_matrixs[0]
        x_data.append(result[0][3])
        y_data.append(result[1][3])
        z_data.append(result[2][3])
        for i in range(len(self.Links)-1):
            result = np.matmul(result, self.transform_matrixs[i+1])
            x_data.append(result[0][3])
            y_data.append(result[1][3])
            z_data.append(result[2][3])

        ax.plot(x_data, y_data, z_data, marker="o", linestyle='-')
        plt.show()

    def Get_Arm_Pos(self, angles):
        self.transform_matrixs = []
        # 各リンクの座標変換行列を取得する
        for i in range(len(self.Links)-1):
            self.transform_matrixs.append(
                self.Links[i].get_link_transform_matrix(np.deg2rad(angles[i])))
        self.transform_matrixs.append(
            self.Links[-1].get_link_transform_matrix())

        print(self.transform_matrixs)

        result = self.transform_matrixs[0]
        for i in range(len(self.Links)-1):
            result = np.matmul(result, self.transform_matrixs[i+1])
        print(result)


if __name__ == "__main__":
    print("Robot Arm 順運動学")
    print("Robot Arm Data")
    np.set_printoptions(suppress=True)
    #link_vector, joint_axis(Roll=0, pitch=1, end_effector=2)
    Link_Datas = [[[0, 0, 10], 0],
                  [[0, 0, 10], 1],
                  [[0, 0, 10], 1],
                  [[0, 0, 5], 2]]
    arm = Arm(Link_Datas)
    arm.Get_Arm_Pos([45, 90, -45])
    arm.Arm_Preview()