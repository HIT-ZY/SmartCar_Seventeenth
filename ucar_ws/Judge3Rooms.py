import os
from math import sqrt

# 路径
import numpy as np
#
# 测试时使用的虚拟房间路径
path_B = r"D:\SmartCar\Xunfei\test_visual_rooms\RoomB"
path_C = r"D:\SmartCar\Xunfei\test_visual_rooms\RoomC"
path_D = r"D:\SmartCar\Xunfei\test_visual_rooms\RoomD"

# 参考概率向量
# 餐厅： 特征：餐具 食物    非特征：桌椅 人
# 客厅： 特征：沙发 电视    非特征：桌椅 宠物
# 卧室： 特征：床          非特征：桌椅 宠物 人

# 特征项
unique_val = 2.0
# 非特征项
common_val = 1.0
# 无关项
outlying_val = -1.0

possibility_vec_kitchen = {"desk_chair": 0.0, "tableware": unique_val, "food": unique_val, "person": common_val,
                           "pet": outlying_val, "bed": outlying_val, "sofa": outlying_val, "tv": outlying_val}
possibility_vec_parlour = {"desk_chair": 0.0, "tableware": outlying_val, "food": outlying_val, "person": outlying_val,
                           "pet": common_val, "bed": outlying_val, "sofa": unique_val, "tv": unique_val}
possibility_vec_bedroom = {"desk_chair": 0.0, "tableware": outlying_val, "food": outlying_val, "person": common_val,
                           "pet": common_val, "bed": unique_val, "sofa": outlying_val, "tv": outlying_val}

# 概率值
possibility_vec_B = {"kitchen": {"p": 0.0, "conf": 0.0}, "parlour": {"p": 0.0, "conf": 0.0},
                     "bedroom": {"p": 0.0, "conf": 0.0}}
possibility_vec_C = {"kitchen": {"p": 0.0, "conf": 0.0}, "parlour": {"p": 0.0, "conf": 0.0},
                     "bedroom": {"p": 0.0, "conf": 0.0}}
possibility_vec_D = {"kitchen": {"p": 0.0, "conf": 0.0}, "parlour": {"p": 0.0, "conf": 0.0},
                     "bedroom": {"p": 0.0, "conf": 0.0}}

# 分类方式
classify = True

# 内置参数
# 便于索引使用的列表和字典
label_list = ["desk_chair", "tableware", "food", "person", "pet", "bed", "sofa", "tv"]
label_dict = {}
room_dict = {0: "B", 1: "C", 2: "D"}
type_dict = {0: "kitchen", 1: "parlour", 2: "bedroom"}

# 是否处于测试阶段
test = False

# 统计向量
statistic_vec_B = {}
statistic_vec_C = {}
statistic_vec_D = {}

statistic_mat = {0: 0, 1: 0, 2: 0}

#
rank_B = np.array([0, 0, 0])
rank_C = np.array([0, 0, 0])
rank_D = np.array([0, 0, 0])


# 创建种类字典、初始化概率向量、统计向量、概率值
def __init(vecB, vecC, vecD):
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
    if test:
        for label in label_list:
            statistic_vec_B[label_dict[label]] = 0.0
            statistic_vec_C[label_dict[label]] = 0.0
            statistic_vec_D[label_dict[label]] = 0.0
    else:
        for label in label_list:
            vecB[label_dict[label]] = 0.0
            vecC[label_dict[label]] = 0.0
            vecD[label_dict[label]] = 0.0

    for i in range(0, 3):
        statistic_mat[i] = 0

    #
    rank_B = np.array([0, 0, 0])
    rank_C = np.array([0, 0, 0])
    rank_D = np.array([0, 0, 0])

    print("Statistic Mats(Vecs) Created!")

    # 概率向量归一化
    print("Normalizing Weights Vecs.")
    normalize(possibility_vec_bedroom)
    normalize(possibility_vec_parlour)
    normalize(possibility_vec_kitchen)
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
    possibility_vector_["kitchen"]["p"] = dot(vec_1=statistic_vector_, vec_2=possibility_vec_kitchen)
    possibility_vector_["parlour"]["p"] = dot(vec_1=statistic_vector_, vec_2=possibility_vec_parlour)
    possibility_vector_["bedroom"]["p"] = dot(vec_1=statistic_vector_, vec_2=possibility_vec_bedroom)

    possibility_vector_["kitchen"]["conf"] = conf_calculation(statistic_vector=statistic_vector_,
                                                              possibility_vector=possibility_vec_kitchen)
    possibility_vector_["parlour"]["conf"] = conf_calculation(statistic_vector=statistic_vector_,
                                                              possibility_vector=possibility_vec_parlour)
    possibility_vector_["bedroom"]["conf"] = conf_calculation(statistic_vector=statistic_vector_,
                                                              possibility_vector=possibility_vec_bedroom)


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
def judge(vecB, vecC, vecD):
    if not test:
        statistic_vec_B = vecB
        statistic_vec_C = vecC
        statistic_vec_D = vecD

    #
    normalize(vecB)
    normalize(vecC)
    normalize(vecD)
    print("\nvec_B:")
    print("\n".join([str(key_) + ":" + str(vecB[key_]) for key_ in vecB.keys()]))
    print("\nvec_C:")
    print("\n".join([str(key_) + ":" + str(vecC[key_]) for key_ in vecC.keys()]))
    print("\nvec_D:")
    print("\n".join([str(key_) + ":" + str(vecD[key_]) for key_ in vecD.keys()]))

    #
    possibility_calculation(statistic_vector_=vecB, possibility_vector_=possibility_vec_B)
    possibility_calculation(statistic_vector_=vecC, possibility_vector_=possibility_vec_C)
    possibility_calculation(statistic_vector_=vecD, possibility_vector_=possibility_vec_D)
    print("\np_vec_B:")
    print("\n".join([str(key_) + ":" + str(possibility_vec_B[key_]) for key_ in possibility_vec_B.keys()]))
    print("\np_vec_C:")
    print("\n".join([str(key_) + ":" + str(possibility_vec_C[key_]) for key_ in possibility_vec_C.keys()]))
    print("\np_vec_D:")
    print("\n".join([str(key_) + ":" + str(possibility_vec_D[key_]) for key_ in possibility_vec_D.keys()]))

    result = [-1, -1, -1]
    result_dict = {}
    possibility_mat = [
        [possibility_vec_B["kitchen"]["p"], possibility_vec_B["parlour"]["p"], possibility_vec_B["bedroom"]["p"]],
        [possibility_vec_C["kitchen"]["p"], possibility_vec_C["parlour"]["p"], possibility_vec_C["bedroom"]["p"]],
        [possibility_vec_D["kitchen"]["p"], possibility_vec_D["parlour"]["p"], possibility_vec_D["bedroom"]["p"]]]
    print("\nP-Mat:")
    p_mat = "\n".join([str(item) for item in possibility_mat])
    print(p_mat)

    conf_mat = [[possibility_vec_B["kitchen"]["conf"], possibility_vec_B["parlour"]["conf"],
                 possibility_vec_B["bedroom"]["conf"]],
                [possibility_vec_C["kitchen"]["conf"], possibility_vec_C["parlour"]["conf"],
                 possibility_vec_C["bedroom"]["conf"]],
                [possibility_vec_D["kitchen"]["conf"], possibility_vec_D["parlour"]["conf"],
                 possibility_vec_D["bedroom"]["conf"]]]
    print("\nC-Mat:")
    c_mat = "\n".join([str(item) for item in conf_mat])
    print(c_mat)

    # sort
    sorted_p_mat = np.argsort(possibility_mat, axis=1)
    print("\nSorted_p_mat:")
    print(sorted_p_mat)

    #
    for i in range(0, 3):
        statistic_mat[sorted_p_mat[i][2]] += 1

    repeat = -1
    for i in range(0, 3):
        if statistic_mat[i] >= 2:
            repeat = i

    # case 1: 无重复
    if repeat == -1:
        print("\nCase 1 :")
        for i in range(0, 3):
            result[i] = sorted_p_mat[i][2]

    # case 2: 存在重复
    else:
        conf_ = [-1, -1, -1]

        # 统计存在重复的部分
        for i in range(0, 3):
            if sorted_p_mat[i][2] == repeat:
                conf_[i] = conf_mat[i][repeat]

        sorted_conf_vec = np.argsort(conf_)

        print("Sorted_conf_vec:")
        print(sorted_conf_vec)

        # 最大置信度者直接认定为成立
        result[sorted_conf_vec[2]] = repeat

        print("\n{Room} was judged as {Type} for highest conf.".format(Room=room_dict[sorted_conf_vec[2]],
                                                                       Type=type_dict[repeat]))

        # 全部重复标识符
        repeat__ = False
        for i in range(0, 3):
            if statistic_mat[i] == 3:
                repeat__ = True

        # case 2-1 两个重复
        # statistic_mat 为【2，1，0】
        # 信任未重复判断
        if not repeat__:
            print("\nCase 2-1:")
            # 直接信任未重复选择
            for i in range(0, 3):
                if statistic_mat[i] == 1:
                    for j in range(0, 3):
                        if sorted_p_mat[j][2] == i and result[j] == -1:
                            result[j] = i
                            print("\n{Room} was judged as {Type} for singular preference.".format(Room=room_dict[j],
                                                                                                  Type=type_dict[i]))

            # 置信度低的重复项被分配为最后一项
            for i in range(0, 3):
                if statistic_mat[i] == 0:
                    for j in range(0, 3):
                        if result[j] == -1 and i not in result:
                            result[j] = i
                            print("\n{Room} was judged as {Type} for lowest conf".format(Room=room_dict[j],
                                                                                         Type=type_dict[i]))

        # case 2-2 三个重复
        # 分析剩余两者概率第二高选择：
        # 2-2-1若选择不同，直接信任
        # 2-2-2若选择相同，按置信度排序
        else:
            # 对可能性第二高的选择统计
            statistic__mat2 = [0, 0, 0]
            for i in range(0, 3):
                if result[i] == -1:
                    statistic__mat2[sorted_p_mat[i][1]] += 1

            # 判断可能性第二高的选择是否重复
            flag_222 = False
            for i in range(0, 3):
                if statistic__mat2[i] == 2:
                    flag_222 = True

            # case 2-2-1
            # 未出现重复
            if not flag_222:
                print("\nCase 2-2-1")
                for i in range(0, 3):
                    if result[i] == -1 and sorted_p_mat[i][1] not in result:
                        result[i] = sorted_p_mat[i][1]
                        print("\n{Room} was judged as {Type} for singular second-preference.".format(Room=room_dict[i],
                                                                                                     Type=type_dict[
                                                                                                         sorted_p_mat[
                                                                                                             i][1]]))

            # case 2-2-2
            # 出现重复，比较置信度
            else:
                print("\nCase 2-2-2")
                conf2 = [-1.0, -1.0, -1.0]

                # 统计置信度
                for i in range(0, 3):
                    if result[i] == -1 and conf_mat[i][1] not in result:
                        conf2[i] = conf_mat[i][1]

                # 排序
                sorted_conf_vec2 = np.argsort(conf2)
                # 高第二置信度
                result[sorted_conf_vec2[2]] = sorted_p_mat[sorted_conf_vec2[2]][1]
                print("\n{Room} was judged as {Type} from second-conf.".format(Room=room_dict[sorted_conf_vec2[2]],
                                                                               Type=type_dict[
                                                                                   sorted_p_mat[sorted_conf_vec2[2]][
                                                                                       1]]))
                # 低第二置信度
                result[sorted_conf_vec2[1]] = sorted_p_mat[sorted_conf_vec2[1]][0]
                print("\n{Room} was judged as {Type} for lowest second-conf".format(Room=sorted_conf_vec2[1],
                                                                                    Type=
                                                                                    sorted_p_mat[sorted_conf_vec2[1]][
                                                                                        0]))
    print("\nResult:")
    print(result)
    for i in range(0, 3):
        result_dict[room_dict[i]] = type_dict[result[i]]
    print(result_dict)
    return result_dict


# 仅供测试使用
if __name__ == "__main__":
    rospy.loginfo("decide to three Room!")
    __init(statistic_vec_B, statistic_vec_C, statistic_vec_D)
    # 初始化概率值
    for key in possibility_vec_B.keys():
        for k_ in possibility_vec_B[key].keys():
            possibility_vec_B[key][k_] = 0.0
    for key in possibility_vec_C.keys():
        for k_ in possibility_vec_C[key].keys():
            possibility_vec_C[key][k_] = 0.0
    for key in possibility_vec_D.keys():
        for k_ in possibility_vec_D[key].keys():
            possibility_vec_D[key][k_] = 0.0

    # 统计
    folder_process(path=path_B, statistic_vec=statistic_vec_B)
    folder_process(path=path_C, statistic_vec=statistic_vec_C)
    folder_process(path=path_D, statistic_vec=statistic_vec_D)

    #
    judge(statistic_vec_B, statistic_vec_C, statistic_vec_D)
