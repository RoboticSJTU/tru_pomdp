import json
from graphviz import Digraph
import pydot

def load_data(json_file_path):
    """解析JSON字符串为Python对象"""
    with open (json_file_path, 'r',encoding='utf8') as fp:
        json_data = json.load(fp)
    return json_data

def visualize_hierarchy(data):
    """生成层级结构树状图"""
    graph = pydot.Dot(graph_type='digraph', rankdir='TB', nodesep='0.5', ranksep='0.5')
    graph.set_node_defaults(shape='box', style='rounded', fontname='Helvetica')

    def add_nodes(parent, node_dict):
        for key in node_dict:
            # 构建节点完整路径
            node_path = f"{parent}.{key}" if parent != 'room' else f"room.{key}"
            
            # 添加节点
            graph.add_node(pydot.Node(node_path, label=key))
            
            # 添加父子关系边
            if parent != 'room':
                graph.add_edge(pydot.Edge(parent, node_path))
            else:
                graph.add_edge(pydot.Edge('room', node_path))  # 处理根节点的特殊情况

            # 递归处理子节点（排除包含属性值的叶子节点）
            value = node_dict[key]
            if isinstance(value, dict) and not ('is_open' in value or 'objects' in value):
                add_nodes(node_path, value)

    # 从根节点开始构建
    graph.add_node(pydot.Node('room', label='room'))
    add_nodes('room', data['room'])
    
    return graph

def visualize_connectivity(data):
    """生成位置节点连通图"""
    graph = pydot.Dot(graph_type='digraph', rankdir='LR', nodesep='1.0', ranksep='2.0')
    graph.set_node_defaults(shape='circle', style='filled', fillcolor='lightblue')
    graph.set_edge_defaults(fontname='Helvetica', fontsize='10')

    # 添加所有位置节点
    poses = list(data['room'].keys())
    for pose in poses:
        graph.add_node(pydot.Node(pose))

    # 添加连接边
    for edge in data['edges']:
        src, dst = edge['link']
        cost = edge['cost']
        # 使用 cost 的倒数作为 weight，使较高的 cost 对应较长的边
        weight = 1.0 / cost if cost != 0 else 1.0
        graph.add_edge(pydot.Edge(src, dst, label=str(cost), weight=str(weight)))

    return graph
    
    return graph

if __name__ == "__main__":
    # 示例JSON数据（替换为实际数据源）
    json_file_path = "kitchen_l_shaped_large.json"
    
    # 加载数据
    data = load_data(json_file_path)
    
    # 生成可视化图表
    hierarchy = visualize_hierarchy(data)
    connectivity = visualize_connectivity(data)
    
    # 使用 pydot 渲染并保存结果
    hierarchy.write_png('hierarchy_l_shaped_large.png')  # 保存层级结构图
    connectivity.write_png('connectivity_l_shaped_large.png')  # 保存连通图