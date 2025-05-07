import numpy as np
from math import cos, pi

class Particle:
    """
    粒子类，用于初始化粒子的位置、速度，并更新它们。
    """

    def __init__(self, bounds, dims):
        lower_bounds, upper_bounds = zip(*bounds)   # 解包bounds列表
        self.position = np.random.uniform(lower_bounds, upper_bounds)  # 初始化粒子位置
        self.velocity = np.random.uniform(-1, 1, size=len(bounds))  # 初始化粒子速度
        self.best_position = np.copy(self.position)  # 初始化粒子历史最优位置
        self.best_fitness = float('inf')  # 初始化粒子历史最优适应度
        self.bounds = bounds

    def update_velocity(self, global_best_position, k, Tmax):
        """
        更新粒子速度的方法。
        global_best_position: 全局最优位置
        w: 惯性权重
        c1: 个体学习因子
        c2: 社会学习因子
        """
        w_min, w_max = 0.4, 0.9
        # w = np.random.uniform(w_min, w_max)
        w = w_max * np.random.rand() * (1-cos(pi*k/(2*Tmax))) + w_min*cos(pi*k/(2*Tmax))
        c1, c2 = 2,2
        r1 = np.random.rand(len(self.bounds))
        r2 = np.random.rand(len(self.bounds))
        cognitive = c1 * r1 * (self.best_position - self.position)  # 个体经验更新
        social = c2 * r2 * (global_best_position - self.position)  # 社会经验更新
        self.velocity = w * self.velocity + cognitive + social  # 速度更新公式

    # 在Particle类中添加混沌扰动方法
    def chaotic_perturbation(self, chaos_type='logistic'):
        if chaos_type == 'logistic':
            # Logistic混沌映射
            chaos_val = 4.0 * np.random.rand() * (1 - np.random.rand())
            perturbation = 0.1 * (chaos_val - 0.5) * (np.array(self.bounds)[:, 1] - np.array(self.bounds)[:, 0])
            self.position += perturbation
        elif chaos_type == 'tent':
            # Tent混沌映射
            x = np.random.rand()
            chaos_val = 2 * x if x < 0.5 else 2 * (1 - x)
            perturbation = 0.1 * (chaos_val - 0.5) * (np.array(self.bounds)[:, 1] - np.array(self.bounds)[:, 0])
            self.position += perturbation
        self.position = np.clip(self.position, *zip(*self.bounds))

    # 修改update_position方法
    def update_position(self, bounds):
        lower_bounds, upper_bounds = zip(*bounds)
        self.position += self.velocity

        # 替换随机偏移为混沌扰动（30%概率）
        if np.random.rand() < 0.3:
            self.chaotic_perturbation(chaos_type='logistic')

        self.position = np.clip(self.position, lower_bounds, upper_bounds)

    '''
    def update_position(self, bounds):
        """
        更新粒子位置的方法。
        bounds: 搜索空间的上下界
        """
        lower_bounds, upper_bounds = zip(*bounds)   # 解包bounds列表
        self.position += self.velocity  # 位置更新公式
        if np.random.rand() < 0.1:  # 10%的概率发生随机偏移
            self.position += np.random.uniform(-100, 100, size=len(bounds))
        self.position = np.clip(self.position, lower_bounds, upper_bounds)  # 限制粒子位置在搜索空间内
    '''

    def update_best_position(self, fitness):
        """
        更新粒子最优位置的方法。
        fitness: 当前适应度值
        """
        if fitness < self.best_fitness:  # 如果当前适应度更好，则更新最优位置和适应度
            self.best_fitness = fitness
            self.best_position = np.copy(self.position)


class PSO:
    """
    粒子群优化算法类。
    """

    def __init__(self, num_particles, bounds, max_iter, fitness_function, dims):
        self.dims = dims    # 问题维数
        self.num_particles = num_particles  # 粒子数量
        self.bounds = bounds  # 搜索空间的上下界
        self.max_iter = max_iter  # 最大迭代次数
        self.particles = [Particle(bounds, dims) for _ in range(num_particles)]  # 初始化粒子群
        self.global_best_position = None  # 全局最优位置
        self.global_best_fitness = float('inf')  # 全局最优适应度
        self.fitness_function = fitness_function  # 适应度函数
        self.stagnation_counter = 0  # 新增：早熟收敛计数器
        self.reset_ratio = 0.3  # 新增：重置比例

    def initialize_global_best(self):
        """
        初始化全局最优解。
        """
        for p in self.particles:
            if p.best_fitness < self.global_best_fitness:
                self.global_best_fitness = p.best_fitness
                self.global_best_position = p.best_position

    # 在PSO类中添加方法
    def detect_stagnation(self, window_size=15, threshold=1e-5):
        """
        改进的早熟检测方法
        :param window_size: 检测窗口大小（最近N代的适应度变化）
        :param threshold: 适应度变化阈值
        """
        if len(self.global_best_fitness_list) < window_size:
            return False

        # 计算窗口内的适应度相对变化率
        recent_fitness = np.array(self.global_best_fitness_list[-window_size:])
        relative_change = np.abs((recent_fitness[1:] - recent_fitness[:-1]) / recent_fitness[:-1])
        return np.mean(relative_change) < threshold

    def optimize(self):
        """
        执行优化过程。
        """
        self.global_best_fitness_list, self.global_best_position_list = [],[]
        for times in range(self.max_iter):
            print(f"正在迭代第{times+1}次")
            for p in self.particles:
                fitness = self.fitness_function(p.position)  # 计算当前位置的适应度
                p.update_best_position(fitness)  # 更新粒子最优位置
                if fitness < self.global_best_fitness:  # 更新全局最优位置
                    self.global_best_fitness = fitness
                    self.global_best_position = p.position
                    lower_bounds, upper_bounds = zip(*self.bounds)  # 解包bounds列表
                    self.global_best_position = np.clip(self.global_best_position, lower_bounds, upper_bounds)
                    self.global_best_position_list.append(self.global_best_position)
                    self.global_best_fitness_list.append(self.global_best_fitness)

            # 每50代检测早熟收敛
            if times % 50 == 0 and self.detect_stagnation():
                print(f"检测到早熟收敛，正在重置部分粒子...")
                for p in np.random.choice(self.particles, size=int(0.3 * self.num_particles)):
                    p.position = np.random.uniform(*zip(*self.bounds))
                    p.velocity = np.random.uniform(-1, 1, size=len(self.bounds))

            for p in self.particles:
                p.update_velocity(self.global_best_position, times, self.max_iter)  # 更新粒子速度
                p.update_position(self.bounds)  # 更新粒子位置

    def get_best_solution(self):
        """
        获取最优解。
        """
        return self.global_best_position, self.global_best_fitness


class PSOMain:
    def __init__(self, dims, bounds, num_particles, max_iter, fitness_func):
        self.dims = dims  # 问题维数
        self.bounds = bounds    # 搜索边界
        self.num_particles = num_particles  # 粒子数
        self.max_iter = max_iter    # 最大迭代次数
        self.fitness_func = fitness_func

    def main(self):
        # 初始化粒子群优化算法
        pso = PSO(self.num_particles, self.bounds, self.max_iter, self.fitness_func, self.dims)
        # 运行优化
        pso.optimize()
        # 获取最优解
        best_position, best_fitness = pso.get_best_solution()
        return best_position, best_fitness

'''
# 定义适应度函数，这里使用 y = x^2 - 1
def fitness_function(x):
    return (x-2)**2 - 2


if __name__ == '__main__':
    # 定义问题的搜索空间的上下界
    bounds = (-10, 10)

    # 初始化粒子群优化算法
    pso = PSO(num_particles=30, bounds=bounds, max_iter=100, fitness_function=fitness_function)

    # 运行优化
    pso.optimize()

    # 获取并打印最优解
    best_position, best_fitness = pso.get_best_solution()
    print(f"最优位置: {best_position}, 最优适应度: {best_fitness}")
'''