"""
计算相关的函数,用np运算返回list
"""
import numpy as np


def mean(data: list) -> float:
    # 计算均值
    return float(np.mean(data))


def var(data: list) -> float:
    # 计算方差
    return float(np.var(data))


def norm(data: np.array) -> np.array:
    # 速度向量单位化
    for i in range(0, data.shape[1]):
        dividend = np.sqrt(data[0, i] ** 2 + data[1, i] ** 2 + data[2, i] ** 2)
        if dividend != 0:
            data[:, i] = data[:, i] / np.sqrt(data[0, i] ** 2 + data[1, i] ** 2 + data[2, i] ** 2)
        else:
            data[:, i] = data[:, i]
    return data


def diff(data: list) -> list:
    # 对数列进行差分，即为求导
    lists_forward = np.array(data)[1:]
    lists = np.array(data)[:-1]
    return list(lists_forward - lists)


def max_percent(data: list, stable_state: float) -> float:
    """
    超调量计算
    :param data: 数据集
    :param stable_state: 稳定状态
    :return: 超调量，小数表示
    """
    # 超调量,若超出稳定状态则记录，若未超出则为负，超调量为0
    return max(float((np.max(data) - stable_state) / stable_state), 0)


def settling_time(data: list, stable_state: float, simple_time: float) -> float:
    """
    调整时间计算，按最后一次5%计算
    :param data: 数据集
    :param stable_state: 稳定状态
    :param simple_time: 采样时间
    :return: 调整时间计算，单位s，若是没找到则系统不稳定，设定为-1
    """
    result = -1
    threshold = stable_state * 0.05
    data = np.array(data)
    data = np.abs(data - stable_state)  # 取绝对值
    final_into_settling = -1  # 最后进入时刻(int)
    for i in range(len(data)):
        # 如果该点在阈值内，且未被记录过，则记录该点
        if final_into_settling == -1 and data[i] <= threshold:
            final_into_settling = i
        # 若之后出现的点超出了阈值，则证明不稳定
        if final_into_settling != -1 and data[i] > threshold:
            final_into_settling = -1
    if final_into_settling != -1:
        result = final_into_settling * simple_time
    return result


def performance_indicator(output_list: list, system_target: float) -> float:
    """
    阶跃性能指标计算，使用了ITAE性能指标
    if y(t) > target:
        result += t * abs(y(t)-target)
    if y(t) < target:
        result += weight * t * abs(y(t)-target)
    :param output_list: 系统输出参数
    :param system_target: 系统信号目标
    :return: 性能指标
    """
    output_list = np.array(output_list)
    error_list = output_list - system_target
    result = 0
    weight = 1.5  # 惩罚权重，用于提升快速性
    for i in range(len(error_list)):
        if error_list[i] > 0:
            result += i * error_list[i]
        else:
            result += - weight * i * error_list[i]
    return result


if __name__ == '__main__':
    a = np.array([[1, 2, 1],
                  [2, 2, 1],
                  [3, 3, 1]],dtype=np.float64)
    print(norm(a))
