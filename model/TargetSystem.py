"""
目标系统，包含一阶系统与二阶系统
"""


class FirstOrderSystem:
    """
    一阶系统
    input > [一阶系统] > output
        Y(s)/R(s) = K/(Ts+1) 使用欧拉方法转化为后向差分
        Y(s)(Ts+1) = KR(s)
        T(Y(k)-Y(k-1)) + Y(k) = KR(k)
        最后得出
        Y(k)=(KR(k)+TY(k-1))/(1+T)
    """

    def __init__(self, K, T):
        self.input = 0  # R(k) 输入
        self.output = 0  # Y(k) 输出
        self.last_output = 0  # Y(k-1) 输出
        self.K = K  # 比例参数
        self.T = T  # 时间常数

    def update(self, input_data):
        """
        :param input_data: 当前输入
        :return: output: Y(k) 输出
        """
        self.input = input_data

        self.output = \
            (self.K * self.input + self.T * self.last_output) \
            / (1 + self.T)


class SecondOrderSystem:
    """
    二阶系统
    input > [二阶系统] > output
        Y(s)/R(s) = K * Wn^2 / (s^2 + 2*damping*Wn* s + Wn^2) 使用欧拉方法转化为后向差分
        最后得出
        Y(k) = (R(k)*K*Wn^2 + 2Y(k-1)(1-2*damping*Wn) - Y(k-2))/(1 + 2*damping*Wn + Wn^2)
    """

    def __init__(self, K, Wn, damping):
        self.input = 0  # R(k) 输入
        self.output = 0  # Y(k) 输出
        self.last_output = 0  # Y(k-1) 输出
        self.second_last_output = 0  # Y(k-2) 输出
        self.K = K  # 比例参数
        self.Wn = Wn  # 自然频率
        self.damping = damping  # 阻尼比

    def update(self, input_data):
        """
        :param input_data: 当前输入
        :return: output: 输出
        """
        self.input = input_data

        self.output = \
            (self.input * self.K * self.Wn ** 2
             + 2 * self.last_output * (1 - 2 * self.damping * self.Wn)
             - self.second_last_output) \
            / (1 + 2 * self.damping * self.Wn + self.Wn ** 2)

        # 更新输出
        self.last_output = self.output
        self.second_last_output = self.last_output

        return self.output
