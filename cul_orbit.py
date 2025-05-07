from init import *
from math import sin, cos, tan, asin, acos, atan, sqrt, degrees, radians, pi, log10
import numpy as np
from PSO_ import *
from geopy.distance import geodesic
from GA import *

# 注：再入角为-5.8时飞船可实现半弹道跳跃式再入
if vessel.control.nodes:
    for nodes in vessel.control.nodes:
        nodes.remove()

s = 12.458  # 梦舟飞船参考面积
r_reenty = 120000 + 6371000  # 再入点矢径

moon_u = moon_orbit.argument_of_periapsis + moon_orbit.true_anomaly  # 月球纬度幅角
moon_lat = asin(sin(moon_orbit.inclination) * sin(moon_u))  # 当前月球赤纬
_ = degrees(moon_lat)

lat_t = radians(lat_t)
lon_t = radians(lon_t)

now_ut = space_center.ut

'''计算轨道'''
class culculate_orbit:
    def __init__(self, node=None, node_orbit_moon=None, node_orbit_earth=None):
        self.start_ut = space_center.ut
        self.node = node
        self.node_orbit_moon = node_orbit_moon
        self.node_orbit_earth = node_orbit_earth

    def create_node(self, node_info, ut):
        node_ut, prograde_dv, normal_dv, radial_dv = node_info[0], node_info[1], node_info[2], node_info[3]
        node = vessel.control.add_node(ut + node_ut, prograde_dv, normal_dv, radial_dv)

    def get_node_orbit(self):
        if body.name == 'Moon':
            self.node = vessel.control.nodes[0]
            self.node_orbit_moon = self.node.orbit
            self.node_orbit_earth = self.node.orbit.next_orbit
        elif body.name == 'Earth':
            self.node = vessel.control.nodes[0]
            self.node_orbit_earth = self.node.orbit

    def cul_orbit(self):
        true_anomaly_at_120 = -acos(((self.node_orbit_earth.semi_major_axis *
                                      (1 - self.node_orbit_earth.eccentricity ** 2) / r_reenty) - 1)
                                    / self.node_orbit_earth.eccentricity)

        gama_at_120 = atan(
            (self.node_orbit_earth.eccentricity * sin(true_anomaly_at_120)) /
            (1 + self.node_orbit_earth.eccentricity * cos(true_anomaly_at_120)))
        gama_at_120 = degrees(gama_at_120)

        u_at_120 = self.node_orbit_earth.argument_of_periapsis + true_anomaly_at_120
        lat_at_120 = asin(sin(self.node_orbit_earth.inclination) * sin(u_at_120))

        inclination = degrees(self.node_orbit_earth.inclination)  # 地心段轨道倾角
        lat_at_periapsis = asin(
            sin(self.node_orbit_earth.inclination) * sin(self.node_orbit_earth.argument_of_periapsis))  # 近地点纬度

        # 出口点偏近点角
        E_at_soi = self.node_orbit_earth.eccentric_anomaly
        # 再入点偏近点角
        E_at_120 = 2 * atan(tan(true_anomaly_at_120 / 2) / sqrt((1 + self.node_orbit_earth.eccentricity)
                                                                / (1 - self.node_orbit_earth.eccentricity)))
        # 到再入点的时间
        theta_120 = -self.node_orbit_earth.true_anomaly_at_radius(r_reenty)
        ut_at_120 = self.node_orbit_earth.ut_at_true_anomaly(theta_120)

        # 再入点经度
        if orbit.body.name == 'Moon':
            lon_at_120 = atan(cos(self.node_orbit_earth.inclination) * tan(
                u_at_120)) + self.node_orbit_earth.longitude_of_ascending_node - \
                         body.orbit.body.rotation_angle - (omiga_earth * (ut_at_120 - self.start_ut))
        elif orbit.body.name == 'Earth':
            lon_at_120 = atan(cos(self.node_orbit_earth.inclination) * tan(
                u_at_120)) + self.node_orbit_earth.longitude_of_ascending_node - \
                         body.rotation_angle - (omiga_earth * (ut_at_120 - self.start_ut))

        lon_at_120 = lon_at_120 % (2*pi)
        # 计算再入航程
        distance_to_target = geodesic((degrees(lat_at_120), degrees(lon_at_120)), (degrees(lat_t), degrees(lon_t))).meters

        return [true_anomaly_at_120, gama_at_120, inclination, u_at_120, lat_at_120, lat_at_periapsis, lon_at_120,
                distance_to_target]

    def delete_node(self):
        self.node.remove()


cul_orbit = culculate_orbit()

'''搜索月地转移轨道'''
class Find_orbit:
    def __init__(self, ut):
        self.ut = ut
        self.search_mode = 1

    '''初步设计轨道'''
    def first_node(self):
        # self.search_mode = input("选择搜索算法\n1:粒子群算法\n2:遗传算法\n3:混合粒子群-遗传算法")
        if int(self.search_mode) == 1:
            # print("搜索算法:粒子群算法")
            bounds = ([1000, 86400 * 1], [830, 900], [-100, 200], [-100, 100])
            pso_prog = PSOMain(4, bounds, 20, 100, self.fitness_func)  # 粒子个数20，最大迭代次数50
            best_position, best_fitness = pso_prog.main()
            print(f"最优位置：{best_position}"
                  f"最优适应度：{best_fitness}")

        elif int(self.search_mode) == 2:
            # print("搜索算法:遗传算法")
            best_position = genetic_algorithm(self.fitness_func)

        elif int(self.search_mode) == 3:
            # print("搜索算法:混合粒子群-遗传算法")
            # 先运行三次粒子群算法，获得初始种群
            self.search_mode = 1
            PSO_init_result = []
            for _ in range(10):
                print(f"\nPSO搜索次数:{_+1}")
                bounds = ([1000, 86400 * 1], [830, 900], [-100, 200], [-100, 100])
                pso_prog = PSOMain(4, bounds, 20, 100, self.fitness_func)  # 粒子个数20，最大迭代次数50
                best_position, best_fitness = pso_prog.main()
                print(best_position)
                PSO_init_result.append(best_position)

            # 切换搜索模式为遗传算法
            self.search_mode = 2
            print("搜索模式:GA")
            best_position = genetic_algorithm(self.fitness_func, PSO_init_result)
            self.search_mode = 1

        node = vessel.control.add_node(now_ut + best_position[0], best_position[1], best_position[2], best_position[3])

        # 判断异常情况(只考虑升轨再入)
        if 700 < sqrt(best_position[1] ** 2 + best_position[2] ** 2 + best_position[3] ** 2) < 1000:
            cul_first_orbit = culculate_orbit(node=node, node_orbit_moon=node.orbit,
                                              node_orbit_earth=node.orbit.next_orbit)
            first_orbit_info = cul_first_orbit.cul_orbit()
            # lat_at_120 = first_orbit_info[4]  # 再入点纬度
            # lat_at_periapsis = first_orbit_info[5]  # 真空近地点纬度
            '''
            if lat_at_120 < lat_at_periapsis:
                status = True
            else:
                status = False
            '''
            status = True
        else:
            status = False
            first_orbit_info = [0, 0, 0, 0, 0, 0]
        return node, status, first_orbit_info

    '''适应度函数'''
    def fitness_func(self, position):
        cul_orbit.create_node(position, self.ut)
        cul_orbit.get_node_orbit()  # 创建机动节点
        try:
            [true_anomaly_at_120, gama_at_120, inclination, u_at_120, lat_at_120, lat_at_periapsis, lon_at_120,
             distance_to_target] = cul_orbit.cul_orbit()  # 计算轨道
        except:
            if int(self.search_mode) == 1:
                true_anomaly_at_120, gama_at_120, inclination, lat_at_120, lat_at_periapsis, lon_at_120, distance_to_target = -10000, -1000000, 100000, 100000, 100000, 10000, 100000000
            elif int(self.search_mode) == 2:
                true_anomaly_at_120, gama_at_120, inclination, lat_at_120, lat_at_periapsis, lon_at_120, distance_to_target = -1e+2, -1e+2, -1e+2, -1e+2, -1e+2, -1e+2, -1e+2

        dv = sqrt(position[1] ** 2 + position[2] ** 2 + position[3] ** 2)

        '''罚函数'''

        def punish(gama_at_120, inclination, lat_at_120, lon_at_120):
            gama_cons = 10 * abs(-gama_at_120 - 5.7)  # 再入角约束
            inclination_cons = 20 * abs(inclination - 42)  # 轨道倾角约束
            if lat_at_120 > lat_at_periapsis:
                punish_val = 100000
            lat_cons = 0.5 * abs(degrees(lat_at_120) - lat_re)  # 再入点纬度约束22.5
            lon_cons = abs(degrees(lon_at_120) - lon_re)  # 再入点经度约束42
            # distance_cons = abs(distance_to_target - 24875000)  # 航程约束
            punish_val = gama_cons + inclination_cons + lat_cons + lon_cons
            return punish_val

        cul_orbit.delete_node()  # 删除机动节点

        if int(self.search_mode) == 1:
            return dv + punish(gama_at_120, inclination, lat_at_120, lon_at_120)
        elif int(self.search_mode) == 2:
            return 1 / (dv + punish(gama_at_120, inclination, lat_at_120, lon_at_120) + 1e-6)

    '''轨道修正'''
    def correct_orbit(self):
        bounds = ([600, 86400*1], [-20, 20], [-50, 50], [-20, 20])
        pso_prog = PSOMain(4, bounds, 20, 100, self.fitness_func)  # 粒子个数20，最大迭代次数50
        best_position, best_fitness = pso_prog.main()
        node = vessel.control.add_node(self.ut + best_position[0], best_position[1], best_position[2], best_position[3])
        if orbit.body.name == 'Moon':
            cul_correct_orbit = culculate_orbit(node, node_orbit_earth=node.orbit.next_orbit, node_orbit_moon=node.orbit)
        elif orbit.body.name == 'Earth':
            cul_correct_orbit = culculate_orbit(node, node_orbit_earth=node.orbit)
        orbit_info = cul_correct_orbit.cul_orbit()
        return node, orbit_info

# 2024年12月30日测试
# 倾角28.973
# 再入点 北纬12 44 24 东经168 24 44 再入角-5.8
# 再入时刻 13：50：11
# 落点 北纬0 0 53 西经168 46 47
# 航程 2887.670公里
# 着陆时刻 14：03：40
# 最大过载5.7G

# 2024年12月31日测试
# 倾角47.542
# 再入点 北纬15 32 47 西经137 33 26 再入角-5.5
# 再入时刻 11:06:11
# 落点 北纬0 0 53 西经168 46 47
# 航程 公里
# 着陆时刻 14：03：40
# 最大过载5.6G
# 失败
