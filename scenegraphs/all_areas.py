import json

def extract_open_areas(scene_graph):
    open_areas = []
    # 遍历每个房间中的cab_front
    for cab_front in scene_graph['room'].values():
        # 遍历每个cab_front下的各个区域
        for area in cab_front.values():
            # 遍历每个区域下的家具项
            for furniture_name, furniture_props in area.items():
                if 'is_open' in furniture_props:
                    open_areas.append(furniture_name)
    return open_areas

# 从文件中读取JSON数据
with open('kitchen_galley.json', 'r') as file:
    data = json.load(file)

# 提取所有具有is_open的区域名称
open_areas_list = extract_open_areas(data)

# 写入txt文件
with open('all_areas_galley.txt', 'w') as file:
    for area_name in open_areas_list:
        file.write(f"{area_name}\n")
