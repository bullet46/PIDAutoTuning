"""
使用PSO粒子群优化算法的PID参数自动整定器
"""
from model.Calculate import norm
from sim_env.Simulation import Simulation
import numpy as np
import time


class AutoTuning:
    def __init__(self):
        self.particle_number = 1000  # 粒子数量设定
        self.total_epoch = 30  # 总迭代次数
        self.speed_inertial = 0.5  # 速度惯性
        self.best_vector_initeral = 0.2  # 粒子最优惯性
        self.global_vector_initeral = 0.3  # 全局最优惯性
        self.iterative_weaken = 0.95  # 迭代步长收敛系数
        self.P_limit = [0, 50]  # P 初始化参数搜寻域
        self.I_limit = [0, 50]  # I 初始化参数搜寻域
        self.D_limit = [-20, 50]  # D 初始化参数搜寻域
        self.P_speed_limit = 1  # P 迭代初始化步长
        self.I_speed_limit = 1  # I 迭代初始化步长
        self.D_speed_limit = 1  # D 迭代初始化步长
        self.now_epoch = 0  # 记录当前迭代次数
        self.initial_list()  # 粒子群列表创建

    def initial_list(self):
        # 初始化PID粒子位置列表，维数: 3 * 总粒子数
        self.particle_position_list = np.random.rand(3, self.particle_number)  # 粒子群初始位置创建为0~1的随机数
        # 映射随机数，产生初始PID数组
        self.particle_position_list[0, :] = self.particle_position_list[0, :] * (self.P_limit[1] - self.P_limit[0]) + \
                                            self.P_limit[0]
        self.particle_position_list[1, :] = self.particle_position_list[1, :] * (self.I_limit[1] - self.I_limit[0]) + \
                                            self.I_limit[0]
        self.particle_position_list[2, :] = self.particle_position_list[2, :] * (self.D_limit[1] - self.D_limit[0]) + \
                                            self.D_limit[0]

        # 初始化PID粒子速度列表，维数: 3 * 总粒子数
        self.particle_vector_list = np.random.rand(3, self.particle_number)
        # 映射初始化速度方向
        self.particle_vector_list[0, :] = (self.particle_vector_list[0, :] - 0.5) * 2 * self.P_speed_limit
        self.particle_vector_list[1, :] = (self.particle_vector_list[1, :] - 0.5) * 2 * self.I_speed_limit
        self.particle_vector_list[2, :] = (self.particle_vector_list[2, :] - 0.5) * 2 * self.D_speed_limit
        self.particle_vector_list = norm(self.particle_vector_list)

        # 初始化PID粒子评价列表 维数: 1 * 总粒子数
        self.particle_loss_list = np.zeros(self.particle_number) + 1e10

        # 初始化粒子个体最优点 维数: 3 * 总粒子数
        self.particle_best_point = np.zeros([3, self.particle_number])
        # 初始化粒子个体最优方向 维数: 3 * 总粒子数
        self.particle_best_vector = np.zeros([3, self.particle_number])

        # 初始化全局最优点 维数: 3 *1
        self.global_best = np.zeros([3, 1])
        self.global_best_loss = 1e10  # 全局最优loss值记录 +
        # 初始化粒子个体全局最优方向 维数: 3 * 总粒子数
        self.global_best_vector = np.zeros([3, self.particle_number])

    def loss_update(self):
        # 计算所有loss
        for i in range(0, self.particle_number):
            P, I, D = self.particle_position_list[:, i]
            simulation = Simulation()
            simulation.pidController.change_pid(P, I, D)
            simulation.start_simulation()
            simulation.calculate_indicator()
            loss = simulation.performance_indicator
            # 若loss小于当前粒子最优值，或者未进行过更新，则进行当前最优更新
            if loss < self.particle_loss_list[i]:
                self.particle_loss_list[i] = loss
                self.particle_best_point[:, i] = np.array([P, I, D]).T
                # 若loss小于全局最优，则认为为全局最优点
                if loss < self.global_best_loss:
                    self.global_best[:, 0] = np.array([P, I, D]).T
                    self.global_best_loss = loss

    def update_vector(self):
        # 更新所有速度向量

        self.particle_best_vector = self.particle_best_point - self.particle_position_list
        self.global_best_vector = self.global_best - self.particle_position_list
        # 标准化
        self.particle_best_vector = norm(self.particle_best_vector)
        self.global_best_vector = norm(self.global_best_vector)

    def position_update(self):
        # 位置迭代
        self.particle_vector_list = self.speed_inertial * self.particle_vector_list \
                                    + self.best_vector_initeral * self.particle_best_vector \
                                    + self.global_vector_initeral * self.global_best_vector

        self.particle_position_list += self.particle_vector_list * self.iterative_weaken

    def start_PSO(self):
        for i in range(self.total_epoch):
            self.now_epoch = i
            print("当前迭代次数:%d" % i)
            a = time.time()
            self.loss_update()
            print("%d 完成,花费时间:", time.time() - a)
            print("当前最优loss:", self.global_best_loss)
            print("P=%f,I=%f,D=%f" % (
                self.global_best[0, 0], self.global_best[1, 0], self.global_best[2, 0]))
            self.update_vector()
            self.position_update()

    def draw(self):
        P, I, D = self.global_best[:, 0]
        simulation = Simulation()
        simulation.pidController.change_pid(P, I, D)
        simulation.start_simulation()
        simulation.calculate_indicator()
        simulation.plot_output()


if __name__ == '__main__':
    autoTuning = AutoTuning()
    autoTuning.start_PSO()
    autoTuning.draw()