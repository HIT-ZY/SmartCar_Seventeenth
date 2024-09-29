import os
from math import sqrt

# 路径
import numpy as np
from std_msgs.msg import Int32

# 房间对应编号：按照拍摄顺序递增
room_dict = {}

# 识别用的权重
# 特征项
unique_val = 2.0
# 非特征项
common_val = 1.0
# 无关项
outlying_val = -1.0

# 参考概率向量
# 餐厅： 特征：餐具 食物    非特征：桌椅 人
# 客厅： 特征：沙发 电视    非特征：桌椅 宠物
# 卧室： 特征：床          非特征：桌椅 宠物 人

possibilityVecKitchen = {"desk_chair": 0.0, "tableware": unique_val, "food": unique_val, "person": common_val,
                         "pet": outlying_val, "bed": outlying_val, "sofa": outlying_val, "tv": outlying_val}
possibilityVecParlour = {"desk_chair": 0.0, "tableware": outlying_val, "food": outlying_val, "person": outlying_val,
                         "pet": common_val, "bed": outlying_val, "sofa": unique_val, "tv": unique_val}
possibilityVecBedroom = {"desk_chair": 0.0, "tableware": outlying_val, "food": outlying_val, "person": common_val,
                         "pet": common_val, "bed": unique_val, "sofa": outlying_val, "tv": outlying_val}

# 概率值
possibilityVec1 = {"kitchen": {"p": 0.0, "conf": 0.0}, "parlour": {"p": 0.0, "conf": 0.0},
                   "bedroom": {"p": 0.0, "conf": 0.0}}
possibilityVec2 = {"kitchen": {"p": 0.0, "conf": 0.0}, "parlour": {"p": 0.0, "conf": 0.0},
                   "bedroom": {"p": 0.0, "conf": 0.0}}

# 内置参数
# 便于索引使用的列表和字典
label_list = ["desk_chair", "tableware", "food", "person", "pet", "bed", "sofa", "tv"]
label_dict = {}
type_dict = {0: "kitchen", 1: "parlour", 2: "bedroom"}

# 是否处于测试阶段
# 测试时使用的虚拟房间路径
path1 = r"D:\SmartCar\Xunfei\test_visual_rooms\RoomB"
path2 = r"D:\SmartCar\Xunfei\test_visual_rooms\RoomC"
test = False

# 统计向量
statisticVec1 = {}
statisticVec2 = {}

statistic_mat = {0: 0, 1: 0, 2: 0}

#
rank1 = np.array([0, 0, 0])
rank2 = np.array([0, 0, 0])


# 创建种类字典、初始化概率向量、统计向量、概率值
def __init(vec1, vec2, room_plan):
    # 初始化输出结果:
    # BC
    if room_plan == Int32(0):
        room_dict[0] = "B"
        room_dict[1] = "C"
        room_dict[2] = "D"
    # BD
    elif room_plan == Int32(1):
        room_dict[0] = "B"
        room_dict[1] = "D"
        room_dict[2] = "C"
    # CD
    elif room_plan == Int32(2):
        room_dict[0] = "C"
        room_dict[1] = "D"
        room_dict[2] = "B"
        print("enter 2")
    print(room_dict)
    # 初始化索引
    print("\n\n\nInit for judging.")
    label_dict["pet"] = "pet"
    label_dict["cat"] = "pet"
    label_dict["dog"] = "pet"
    label_dict["bird"] = "pet"
    label_dict["mouse"] = "pet"
    label_dict["nutshell"] = "food"
    label_dict["food"] = "food"
    label_dict["apple"] = "food"
    label_dict["burger"] = "food"
    label_dict["noodle"] = "food"
    label_dict["tv"] = "tv"
    label_dict["tv1"] = "tv"
    label_dict["tv2"] = "tv"
    label_dict["tv3"] = "tv"
    label_dict["tv4"] = "tv"
    label_dict["bed"] = "bed"
    label_dict["bed1"] = "bed"
    label_dict["bed2"] = "bed"
    label_dict["bed3"] = "bed"
    label_dict["bed4"] = "bed"
    label_dict["sofa"] = "sofa"
    label_dict["sofa1"] = "sofa"
    label_dict["sofa2"] = "sofa"
    label_dict["sofa3"] = "sofa"
    label_dict["sofa4"] = "sofa"
    label_dict["tableware"] = "tableware"
    label_dict["fork_spoon"] = "tableware"
    label_dict["spoon"] = "tableware"
    label_dict["plate"] = "tableware"
    label_dict["fork"] = "tableware"
    label_dict["teapot"] = "tableware"
    label_dict["desk_chair"] = "desk_chair"
    label_dict["desk_chair1"] = "desk_chair"
    label_dict["desk_chair2"] = "desk_chair"
    label_dict["desk_chair3"] = "desk_chair"
    label_dict["desk_chair4"] = "desk_chair"
    label_dict["person"] = "person"

    print("\nLabel_dict:")
    for _key in label_dict.keys():
        print(str(_key) + ": " + str(label_dict[_key]))
    print("\nLabel_Dict Created!")

    # 初始化统计向量
    print("Init Statistic Mats(Vecs).")

    for label in label_list:
        vec1[label_dict[label]] = 0.0
        vec2[label_dict[label]] = 0.0

    for i in range(0, 3):
        statistic_mat[i] = 0

    #
    rank1 = np.array([0, 0, 0])
    rank2 = np.array([0, 0, 0])

    print("Statistic Mats(Vecs) Created!")

    # 概率向量归一化
    print("Normalizing Weights Vecs.")
    normalize(possibilityVecBedroom)
    normalize(possibilityVecKitchen)
    normalize(possibilityVecParlour)
    print("Weights Vecs Normalized.")


# 统计指定房间的概率，保存到current_vector
def statistic(statistic_vector, label, val, xywh):
    flag_legal = True
    # 删除不合理边框
    width_thre = 0.95
    height_thre = 0.05
    val_thre = 1.0
    if xywh[2] > width_thre and val < val_thre:
        if 0 < xywh[1] - 0.5 * xywh[3] < height_thre or 1 > xywh[1] + 0.5 * xywh[3] > 1 - height_thre:
            flag_legal = False
    if flag_legal:
        statistic_vector[label_dict[label]] += val
    else:
        print("Delete {label} for unreasonable rect position: x:{x} y:{y} w:{w} h:{h}".format(label=label,
                                                                                              x="%.4f" % xywh[0],
                                                                                              y="%.4f" % xywh[1],
                                                                                              w="%.4f" % xywh[2],
                                                                                              h="%.4f" % xywh[3]))


# 向量点乘
def dot(vec_1, vec_2):
    result = 0.0
    for _key in vec_1.keys():
        result += float(vec_1[_key]) * float(vec_2[_key])
    return result


# 向量归一(100)化
def normalize(vec):
    eps = 1e-6
    square_sum = 0.0
    for _key in vec.keys():
        square_sum += vec[_key] ** 2

    if abs(square_sum) > eps:
        for _key in vec.keys():
            vec[_key] /= sqrt(square_sum)
            vec[_key] *= 100.0


# 统计概率
def possibility_calculation(statistic_vector_: dict, possibility_vector_: dict):
    possibility_vector_["kitchen"]["p"] = dot(vec_1=statistic_vector_, vec_2=possibilityVecKitchen)
    possibility_vector_["parlour"]["p"] = dot(vec_1=statistic_vector_, vec_2=possibilityVecParlour)
    possibility_vector_["bedroom"]["p"] = dot(vec_1=statistic_vector_, vec_2=possibilityVecBedroom)

    possibility_vector_["kitchen"]["conf"] = conf_calculation(statistic_vector=statistic_vector_,
                                                              possibility_vector=possibilityVecKitchen)
    possibility_vector_["parlour"]["conf"] = conf_calculation(statistic_vector=statistic_vector_,
                                                              possibility_vector=possibilityVecParlour)
    possibility_vector_["bedroom"]["conf"] = conf_calculation(statistic_vector=statistic_vector_,
                                                              possibility_vector=possibilityVecBedroom)


# 统计置信度
def conf_calculation(statistic_vector, possibility_vector):
    eps = 1e-6
    inf = 1e6
    neg_weight = 0.0
    total_weight = 0.0
    for key in possibility_vector.keys():
        total_weight += statistic_vector[key] * possibility_vector[key]
        if possibility_vector[key] < 0.0:
            neg_weight += statistic_vector[key] * possibility_vector[key]
    if abs(neg_weight) > eps:
        return abs(total_weight / neg_weight)
    else:
        return inf


# 文件夹操作
# 读取文件夹下所有txt文件并进行文件级统计函数调用
def folder_process(path, statistic_vec):
    file_lst = os.listdir(path)
    for file in file_lst:
        file_process(file=path + "\\" + file, statistic_vec=statistic_vec)


# 文件级统计
def file_process(file, statistic_vec):
    # 供测试用的假参数
    fake_list = [0, 0, 0, 0]

    with open(file, encoding="utf-8") as f:
        lines = f.readlines()
        for line in lines:
            content = line.split(" ")
            category = content[0]
            possibility = content[1]
            statistic(statistic_vector=statistic_vec, label=category, val=float(possibility), xywh=fake_list)


# 判断类型
def judge(vec1, vec2):
    # 统计值归一化
    normalize(vec1)
    normalize(vec2)
    print("\nvec1:")
    print("\n".join([str(key_) + ":" + str(vec1[key_]) for key_ in vec1.keys()]))
    print("\nvec2:")
    print("\n".join([str(key_) + ":" + str(vec2[key_]) for key_ in vec2.keys()]))

    # 计算概率值和置信度
    possibility_calculation(statistic_vector_=vec1, possibility_vector_=possibilityVec1)
    possibility_calculation(statistic_vector_=vec2, possibility_vector_=possibilityVec2)
    print("\np_vec_1:")
    print("\n".join([str(key_) + ":" + str(possibilityVec1[key_]) for key_ in possibilityVec1.keys()]))
    print("\np_vec_2:")
    print("\n".join([str(key_) + ":" + str(possibilityVec2[key_]) for key_ in possibilityVec2.keys()]))

    result = [-1, -1, -1]
    result_dict = {}
    possibility_mat = [
        [possibilityVec1["kitchen"]["p"], possibilityVec1["parlour"]["p"], possibilityVec1["bedroom"]["p"]],
        [possibilityVec2["kitchen"]["p"], possibilityVec2["parlour"]["p"], possibilityVec2["bedroom"]["p"]]]

    print("\nP-Mat:")
    p_mat = "\n".join([str(item) for item in possibility_mat])
    print(p_mat)

    conf_mat = [[possibilityVec1["kitchen"]["conf"], possibilityVec1["parlour"]["conf"],
                 possibilityVec1["bedroom"]["conf"]],
                [possibilityVec2["kitchen"]["conf"], possibilityVec2["parlour"]["conf"],
                 possibilityVec2["bedroom"]["conf"]]]

    print("\nC-Mat:")
    c_mat = "\n".join([str(item) for item in conf_mat])
    print(c_mat)

    # sort
    sorted_p_mat = np.argsort(possibility_mat, axis=1)
    print("\nSorted_p_mat:")
    print(sorted_p_mat)

    #
    for i in range(0, 2):
        statistic_mat[sorted_p_mat[i][2]] += 1

    repeat = -1
    for i in range(0, 2):
        if statistic_mat[i] == 2:
            repeat = i

    # case 1: 无重复
    if repeat == -1:
        print("\nCase 1 :")
        for i in range(0, 2):
            result[i] = sorted_p_mat[i][2]

    # case 2: 存在重复
    else:
        conf_ = [-1, -1]

        # 统计存在重复的部分
        for i in range(0, 2):
            if sorted_p_mat[i][2] == repeat:
                conf_[i] = conf_mat[i][repeat]

        sorted_conf_vec = np.argsort(conf_)

        print("Sorted_conf_vec:")
        print(sorted_conf_vec)

        # 最大置信度者直接认定为成立
        result[sorted_conf_vec[1]] = repeat

        print("\n{Room} was judged as {Type} for highest conf.".format(Room=room_dict[sorted_conf_vec[1]],
                                                                       Type=type_dict[repeat]))

        # 置信度较低者选择第二高可能性结果
        for i in range(0, 2):
            if result[i] == -1:
                result[i] = sorted_p_mat[i][1]

    # 未拍照房间自动选择剩余类型
    for i in range(0, 3):
        if i not in result:
            result[2] = i

    print("\nResult:")
    print(result)
    for i in range(0, 3):
        result_dict[room_dict[i]] = type_dict[result[i]]
    print(result_dict)
    return result_dict


# 仅供测试使用
if __name__ == "__main__":
    __init(statisticVec1, statisticVec2)

    # 初始化概率值
    for key in possibilityVec1.keys():
        for k_ in possibilityVec1[key].keys():
            possibilityVec1[key][k_] = 0.0
    for key in possibilityVec2.keys():
        for k_ in possibilityVec2[key].keys():
            possibilityVec2[key][k_] = 0.0

    # 统计
    folder_process(path=path1, statistic_vec=statisticVec1)
    folder_process(path=path2, statistic_vec=statisticVec2)

    #
    judge(statisticVec1, statisticVec2)
