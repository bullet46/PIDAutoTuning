class PIDController:
    """
    PID控制器
    """

    def __init__(self, Kp, Ki, Kd):
        self.error = 0  # 控制器输入误差输入
        self.output = 0  # 控制器输出
        self.last_error = 0  # k-1 时刻控制器误差输入
        self.total_error = 0  # 以往控制器误差总和
        self.Kp = Kp  # Kp 比例参数
        self.Ki = Ki  # Ki 积分参数
        self.Kd = Kd  # Kd 微分参数

    def change_pid(self, Kp, Ki, Kd):
        """
        更改当前PID控制器的PID参数值
        """
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

    def update(self, input_error):
        """
        :param input_error: k时刻误差输出更新
        :return output : 控制量
        """
        self.total_error += input_error  # 增加累计误差

        # 计算输出
        self.output = \
            self.Kp * (input_error + \
                       self.Ki * self.total_error + \
                       self.Kd * (input_error - self.last_error))

        # 重设k-1时刻输入记忆
        self.last_error = input_error
