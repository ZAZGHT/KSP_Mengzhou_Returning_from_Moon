import random
import math

# 遗传算法参数
POPULATION_SIZE = 100  # 种群大小
GENE_LENGTH = 4  # 基因长度（每个个体有4个参数）
GENERATIONS = 15  # 迭代代数
MUTATION_RATE = 2  # 初始变异率
CROSSOVER_RATE = 2  # 交叉率
ELITE_SIZE = 10  # 精英保留数量
IMMIGRATION_RATE = 0.1  # 移民策略的比例
TOURNAMENT_SIZE = 5  # 锦标赛规模

# 变量范围
bounds = ([1000, 86400 * 1], [800, 860], [-30, 30], [-30, 30])


# 初始化种群
def initialize_population(pso_results=None):
    population = []
    if pso_results is not None:
        for position in pso_results:
            individual = [float(x) for x in position]  # 确保位置是浮点数
            population.append(individual)
            # 如果PSO结果不足，随机生成剩余部分
        while len(population) < POPULATION_SIZE:
            individual = [random.uniform(bounds[i][0], bounds[i][1]) for i in range(GENE_LENGTH)]
            population.append(individual)
    else:
        for _ in range(POPULATION_SIZE):
            individual = [
                random.uniform(bounds[i][0], bounds[i][1]) for i in range(GENE_LENGTH)
            ]
            population.append(individual)
    return population



# 选择操作：轮盘赌选择
def selection(population, fitnesses):
    total_fitness = sum(fitnesses)
    probabilities = [f / total_fitness for f in fitnesses]
    selected_indices = random.choices(range(len(population)), probabilities, k=len(population))
    return [population[i] for i in selected_indices]

# 锦标赛选择
def tournament_selection(population, fitnesses, tournament_size):
    selected = []
    for _ in range(len(population)):
        # 随机选择 tournament_size 个个体
        tournament = random.sample(list(zip(population, fitnesses)), tournament_size)
        # 选择适应度最高的个体
        winner = max(tournament, key=lambda x: x[1])[0]
        selected.append(winner)
    return selected

# 交叉操作：均值交叉
def crossover(parent1, parent2):
    if random.random() < CROSSOVER_RATE:
        child1 = [(parent1[i] + parent2[i]) / 2 for i in range(GENE_LENGTH)]
        child2 = [(parent2[i] + parent1[i]) / 2 for i in range(GENE_LENGTH)]
        return child1, child2
    else:
        return parent1, parent2


# 变异操作：随机扰动
def mutate(individual):
    for i in range(GENE_LENGTH):
        if random.random() < MUTATION_RATE:
            individual[i] += random.uniform(-1, 1)
            individual[i] = min(max(individual[i], bounds[i][0]), bounds[i][1])
    return individual


# 主函数
def genetic_algorithm(fitness_func, pso_results=None):
    if pso_results is not None:
        population = initialize_population(pso_results)
    else:
        population = initialize_population()

    for generation in range(GENERATIONS):
        fitnesses = [fitness_func(individual) for individual in population]

        # 精英保留
        elite_indices = sorted(range(len(fitnesses)), key=lambda i: fitnesses[i], reverse=True)[:ELITE_SIZE]
        elite = [population[i] for i in elite_indices]

        # 锦标赛选择
        selected_population = tournament_selection(population, fitnesses, TOURNAMENT_SIZE)

        # 选择操作
        selected_population = selection(population, fitnesses)

        # 交叉和变异操作
        new_population = elite  # 添加精英个体
        for i in range(0, len(selected_population) - ELITE_SIZE, 2):
            parent1, parent2 = selected_population[i], selected_population[i + 1]
            child1, child2 = crossover(parent1, parent2)
            new_population.append(mutate(child1))
            new_population.append(mutate(child2))

        # 移民策略：随机替换一部分个体
        num_immigrants = int(POPULATION_SIZE * IMMIGRATION_RATE)
        for _ in range(num_immigrants):
            new_population[random.randint(0, POPULATION_SIZE - 1)] = initialize_population()[0]

        population = new_population

        # 输出当前代的最佳适应度
        best_individual = max(population, key=fitness_func)
        best_fitness = fitness_func(best_individual)
        print(f"Generation {generation}: Best Fitness = {best_fitness}, Best Individual = {best_individual}")

    # 输出最终结果
    best_individual = max(population, key=fitness_func)
    best_fitness = fitness_func(best_individual)
    print(f"Best individual: {best_individual}")
    print(f"Best fitness: {best_fitness}")

    return best_individual


# 运行遗传算法
# genetic_algorithm()