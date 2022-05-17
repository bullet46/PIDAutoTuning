from model import PID, TargetSystem, Calculate
import matplotlib.pyplot as plt
import numpy as np
import threading
import time


class Simulation:
    """
    模拟控制系统
                       Kp     Ki    Kd
                       ↓      ↓      ↓
    target - > error > [PID_Controller] > [Target_System] > output
           ↑                                               ↓
           -------------------------------------------------
    """

    def __init__(self):
        self.input_data_list = [0]  # 初始化输入列表
        self.output_data_list = [0]  # 初始化输出列表
        self.error_data_list = [0]  # 初始化误差列表
        self.pid_output_list = [0]  # 初始化pid输出列表
        self.simple_time = 0.01  # 采样时间默认为0.01
        self.end_time = 1  # 结束时间为1s
        self.timer = 0  # 采样计时器
        self.system_target = 1  # 设置当前目标值为1
        self.total_simple_number = int(self.end_time / self.simple_time)  # 总计采样次数
        self.pidController = PID.PIDController(5, 50, 0)  # 初始化PID控制器
        self.targetSystem1 = TargetSystem.FirstOrderSystem(1, 10)  # 初始化一阶系统1
        self.targetSystem2 = TargetSystem.FirstOrderSystem(2, 5)  # 初始化一阶系统2
        self.targetSystem3 = TargetSystem.SecondOrderSystem(1, 10, 0.6)  # 初始化二阶系统3
        self.performance_indicator = 10000000  # 性能指标，越小越好

    def next_time(self, system_target):
        """
        系统状态更新
        :param system_target: 当前目标值
        """

        self.input_data_list.append(system_target)  # 更新输入列表
        self.error_data_list.append(self.input_data_list[-1] - self.output_data_list[-1])
        self.pidController.update(self.input_data_list[-1] - self.output_data_list[-1])  # 更新pid控制器

        self.pid_output_list.append(self.pidController.output)
        self.targetSystem1.update(self.pidController.output)  # 更新目标系统1 1阶惯性
        self.targetSystem2.update(self.targetSystem1.output)  # 更新目标系统2 1阶惯性
        self.targetSystem3.update(self.targetSystem2.output)  # 更新目标系统3 2阶阻尼
        self.output_data_list.append(self.targetSystem3.output)  # 更新输出列表
        self.timer += 1  # 更新仿真步数

    def plot_output(self):
        # 绘制输出
        # 绘制输入输出图像
        plt.figure()
        time_list = [num for num in np.arange(0, self.end_time, self.simple_time)]
        plt.plot(time_list, self.output_data_list, label="output")
        plt.plot(time_list, self.input_data_list, label="input")
        plt.legend(loc='best')
        # 绘制PID控制器输出与误差曲线
        plt.figure()
        time_list = [num for num in np.arange(0, self.end_time, self.simple_time)]
        plt.plot(time_list, self.pid_output_list, label="pid_output")
        plt.plot(time_list, self.error_data_list, label="error")
        plt.legend(loc='best')

        plt.show()

    def print_quota(self):
        # 打印所有性能指标
        print("该系统调整时间为:", end="")
        print(Calculate.settling_time(self.output_data_list, self.system_target, self.simple_time), end="")
        print("s")
        print("该系统超调量为:", end="")
        print(Calculate.max_percent(self.output_data_list, self.system_target) * 100, end="")
        print("%")
        print("该系统IITAE参数指标为:", end="")
        print(self.performance_indicator)

    def start_simulation(self):
        # 开始仿真
        while self.timer < self.total_simple_number - 1:
            # 阶跃响应从第10个采样点开始
            if self.timer <= 10:
                input_data = 0
            else:
                input_data = self.system_target
            self.next_time(input_data)

    def calculate_indicator(self):
        # 计算改进的ITAE性能指标，作为系统控制性能参考指标
        self.performance_indicator = Calculate.performance_indicator(self.output_data_list, self.system_target)


if __name__ == '__main__':
    a = time.time()
    simulation = Simulation()
    simulation.pidController.change_pid(0.805735, 42.191594, -7.834392)
    simulation.start_simulation()
    simulation.calculate_indicator()
    print("计算时间为:%fs" % (time.time() - a))
    simulation.print_quota()
    simulation.plot_output()
