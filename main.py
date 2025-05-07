import time
from Function import *
from geopy.distance import geodesic


print(f"当前飞船：{vessel.name}")
p = Do_Once()

node_status = 0     # 机动节点执行状态

'''主程序'''
firstNode, first_info = find_first_orbit()  # 第一次搜索月地转移轨道
while True:  # 执行月地转移入射机动循环
    deside = input("是否执行机动节点?(y/n)")
    if deside == 'y':
        node_status = execute_node(firstNode, node_status)
        if node_status == 1:
            print("月地转移入射机动结束\n")
            node_status = 0
            break
    elif deside == 'n':
        if vessel.control.nodes:
            for nodes in vessel.control.nodes:
                nodes.remove()
        firstNode, first_info = find_first_orbit()  # 第一次搜索月地转移轨道

while True:     # 执行中途修正机动循环
    correct_deside = input("是否进行中途修正?(y/n)")
    if correct_deside == 'y':
        now_ut_correct = space_center.ut
        correctOrbit = Find_orbit(now_ut_correct)
        correctNode, correct_info = correctOrbit.correct_orbit()  # 搜索修正轨道
        gama_at_120, correct_inclination, lat_at_120, lon_at_120, distance_to_target = \
            correct_info[1], correct_info[2], correct_info[4], correct_info[6], correct_info[7]
        print(f"\n计算完成！\n"
              f"再入角:{gama_at_120}°\n"
              f"地心段轨道倾角:{correct_inclination}°\n"
              f"再入点纬度:{degrees(lat_at_120)}\n"
              f"再入点经度:{degrees(lon_at_120)}\n"
              f"再入航程:{distance_to_target / 1000}km\n")
        while True:
            deside = input("是否执行机动节点?(y/n)")
            if deside == 'y':
                node_status = execute_node(correctNode, node_status)  # 执行机动节点
                if node_status == 1:
                    print("中途修正结束\n")
                    node_status = 0
                    break
            elif deside == 'n':
                if vessel.control.nodes:
                    for nodes in vessel.control.nodes:
                        nodes.remove()
                correctNode, correct_info = correctOrbit.correct_orbit()  # 搜索修正轨道
                gama_at_120, correct_inclination, lat_at_120, lon_at_120, distance_to_target = \
                    correct_info[1], correct_info[2], correct_info[4], correct_info[6], correct_info[7]
                print(f"\n计算完成！\n"
                      f"再入角:{gama_at_120}°\n"
                      f"地心段轨道倾角:{correct_inclination}°\n"
                      f"再入点纬度:{degrees(lat_at_120)}\n"
                      f"再入点经度:{degrees(lon_at_120)}\n"
                      f"再入航程:{distance_to_target / 1000}km\n")
    elif correct_deside == 'n':
        break

while input("是否可以时间加速?(y/n)") == "y":
    break
print("正在加速至出口点")
vessel = space_center.active_vessel
orbit = vessel.orbit
dt_to_soi = orbit.time_to_soi_change
space_center.warp_to(space_center.ut+dt_to_soi)     # 加速至出口点

'''加速时间至5000公里高度'''
vessel.control.sas = True
vessel.control.sas_mode = vessel.control.sas_mode.retrograde  # 设置为逆行姿态
while input("是否可以时间加速?(y/n)") == "y":
    break
space_center = conn.space_center    # 重新获取space_center
vessel = space_center.active_vessel
orbit = vessel.orbit
body = orbit.body
if body.name == 'Earth':
    theta_5000 = -orbit.true_anomaly_at_radius(5000000+6371000)
    ut_at_5000 = orbit.ut_at_true_anomaly(theta_5000)
elif body.name == 'Moon':
    theta_5000 = -orbit.next_orbit.true_anomaly_at_radius(5000000+6371000)
    ut_at_5000 = orbit.next_orbit.ut_at_true_anomaly(theta_5000)
space_center.warp_to(ut_at_5000)    # 加速到5000公里高度
mass_before_seperate = vessel.mass
vessel.control.activate_next_stage()
print('服-返分离')
if vessel.mass == mass_before_seperate:
    vessel.control.activate_next_stage()

'''加速时间到156公里高度'''
while input("是否可以时间加速?(y/n)") == "y":
    break
theta_at_156 = -orbit.true_anomaly_at_radius(156000+6371000)    # 156公里高度的真近点角
ut_at_156 = orbit.ut_at_true_anomaly(theta_at_156)
space_center.warp_to(ut_at_156)

'''再入段飞行程序'''
control_mode = "manual"    # 默认再入阶段控制模式（自动控制）
guidance_mode = "Predict_Correct_Guidance"   # 制导模式
# p_c = Predict_Corrector(lat_t, lon_t, [])  # 制导对象
reentry_times = 0   # 再入次数
# from predict_correct_test import *
while True:
    mean_altitude = vessel.flight().mean_altitude   # 海拔高度
    gama = gama_at_120 = atan(
            (orbit.eccentricity * sin(orbit.true_anomaly)) /
            (1 + orbit.eccentricity * cos(orbit.true_anomaly)))     # 飞行路径角
    g_force = vessel.flight().g_force   # 过载

    if 115000 < mean_altitude < 120000:
        p.Print("飞船一次再入","飞船第一次再入")
        vessel.control.sas_mode = vessel.control.sas_mode.stability_assist
        time.sleep(4)

    elif 20000 < mean_altitude <= 100000:
        orbit = vessel.orbit
        if orbit.time_to_periapsis < 2:
            p.Print("再入最低点","再入最低点")
            print(f"高度{format(vessel.flight().mean_altitude / 1000, '.3f')}km")
            time.sleep(2)
        if orbit.time_to_apoapsis < 2:
            p.Print("跳出最高点","跳出最高点")
            print(f"高度{format(vessel.flight().mean_altitude / 1000, '.3f')}km")
            reentry_times = 1
            time.sleep(2)
        if g_force > 0.2 and reentry_times == 1:
            p.Print("飞船第二次再入","飞船第二次再入")

    elif 9800 < vessel.flight().surface_altitude <= 10000:
        p.Print("制导结束", "制导结束")
        vessel.control.rcs = False  # 关闭RCS
        vessel.control.sas = False  # 关闭SAS
        vessel.control.toggle_action_group(2)   # 开减速伞
        # vessel.control.activate_next_stage()  # 启动下一分级，开伞
        p.Print("开减速伞", "开减速伞")
        time.sleep(2)

    elif 1100 < vessel.flight().surface_altitude <= 1200:
        # vessel.control.activate_next_stage()    # 启动下一分级，开伞
        vessel.control.toggle_action_group(3)   # 开主伞
        p.Print("开主伞", "开主伞")
        time.sleep(2)

    elif 450 < vessel.flight().surface_altitude <= 500:
        # vessel.control.activate_next_stage()  # 启动下一分级，抛防热大底
        vessel.control.toggle_action_group(4)   # 抛防热大底
        p.Print("抛大底", "抛防热大底")
        time.sleep(20)

    elif 1 < vessel.flight().surface_altitude < 2:
        break

lat = vessel.flight().latitude
lon = vessel.flight().longitude
distance = geodesic((lat, lon), (degrees(lat_t), degrees(lon_t))).meters
print(f"返回舱着陆\n"
      f"落点纬度：{lat}\n"
      f"落点经度：{lon}\n"
      f"落点偏差：{format(distance / 1000, '.3f')}km\n")
