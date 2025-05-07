import krpc
from math import atan, cos, tan, degrees, radians


conn = krpc.connect(name="test")
space_center = conn.space_center
vessel = space_center.active_vessel
# print(vessel.name)

orbit = vessel.orbit
body = orbit.body  # 月球
# print(body.name)
moon_orbit = body.orbit

miu_earth = 6.67e-11 * 5.972e+24    # 地球引力常数
miu_moon = 6.67e-11 * 7.342e+22     # 月球引力常数
omiga_earth = 7.292115e-5   # 地球自转速度

location_target = {"东风着陆场-酒泉卫星发射中心": [41.117778, 100.468611],
                   "四子王旗着陆场": [42.358889, 111.408056],
                   "文昌航天发射场": [19.613611, 110.955278]}

lat_t, lon_t = location_target["东风着陆场-酒泉卫星发射中心"][0], location_target["东风着陆场-酒泉卫星发射中心"][1]
lat_re = 22.5   # 再入点纬度18 22.5  28
lon_re = 42     # 再入点经度57 42  75
# lat_t = 42.358889
# lon_t = 111.408056

'''
22 22 47
-151 39 38
9 50 06
-121 12 29
'''


start_ut = space_center.ut