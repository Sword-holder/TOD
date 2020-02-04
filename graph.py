import matplotlib.pyplot as plt
import math
from ue.model import TrafficFlowModel
import copy
INF = 10000000000

# 全局参数
param = {}

class Node(object):
    def __init__(self, id, type, TAZ=None, area=None, density=None, E=None, L=None, D=None, LT=None, De=None):
        # id是节点的唯一标识符
        self.id = id
        # type有ordinary、candidate、origins、destination
        self.type = type
        # TAZ有None、AOZ、TSA
        self.TAZ = TAZ
        # 点的面积
        self.area = area
        # 区域的密度
        self.density = density
        #
        self.H = None
        # h表示建设TOD或PNR后人口的变化
        self.h_tod = None
        self.h_pnr = None
        # 到终点的交通阻抗
        self.T = None
        self.T_road = None
        self.T_metro = None
        # 
        self.A = None
        # 就业岗位数
        self.E = E
        # 
        self.L = L
        self.D = D
        # 
        self.LT = LT
        self.De = De

    def __str__(self):
        return 'Node ' + str(self.id) + ': type = ' + self.type

class Edge(object):
    def __init__(self, source, target, type, stations_num=None, capacity=None, free_time=None, length=None):
        self.source = source
        self.target = target
        # type有roadway、access、metro
        self.type = type
        # 初始化出行的花费值
        self.cost = None
        # 最大容量
        self.capacity = capacity
        if type == 'roadway':
            self.cost = param['tl0']
        elif type == 'access':
            self.cost = param['tp0']
        elif type == 'metro':
            self.cost = param['t_hat_operate_metro'] * stations_num + \
                        param['t_hat_wait_metro'] + param['t_hat_walk']
            self.cost = round(self.cost, 3)

        self.length = length
        self.flow = 0
        self.link_time = 0

    def __str__(self):
        return 'Edge: source = ' + str(self.source) + ', target = ' + str(self.target) +\
             ', type = ' + self.type + ', cost = ' + str(self.cost)

class Graph(object):
    def __init__(self, nodes, edges, mode=None):
        # if mode != 'PNR':
        #     edges = [e for e in edges if e.type != 'access']
        self.nodes = nodes
        self.edges = edges
        # 初始化node查找表，通过节点id找节点
        self.nodes_map = {}
        for n in nodes:
            self.nodes_map[n.id] = n
        # 初始化邻接表
        self.adj_table = {}
        for n in nodes:
            self.adj_table[n.id] = []
        for e in edges:
            self.adj_table[e.source].append(e)
        # 计算模式，有None（表示baseline）、TOD（表示TOD）、PNR（表示P+R）
        self.mode = mode


    def RRM(self):
        destinations = self.get_destinations()
        # 初始化TAZ的属性
        AOZ_node = self.get_AOZ_node()
        TSA_node = self.get_TSA_node()

        TSA_node.A = sum([self.nodes_map[d.id].E / TSA_node.T[d.id] for d in destinations])         
        AOZ_node.A = sum([self.nodes_map[d.id].E / AOZ_node.T[d.id] for d in destinations])
        # 获得出发点
        origin_nodes = self.get_origins()
        # 初始化TOD的h值
        for n in origin_nodes:
            n.h_tod = self.init_h(n)
            # 初始化PNR的h值
            n.h_pnr = -param['Lp'] * n.D
            # 更新H值
            # n.H += n.h_tod

        delta_h = 0
        if self.mode == 'TOD':
            delta_h = TSA_node.h_tod
        elif self.mode == 'PNR':
            delta_h = TSA_node.h_pnr
        TSA_node.H += delta_h
        AOZ_node.H -= delta_h



    def TDM(self):
        origin_nodes = self.get_origins()
        destinations = self.get_destinations()

        self.Ds = [] # 这个是需求量
        
        for origin in origin_nodes:
            divisor = sum([self.nodes_map[d.id].E * math.exp(-param['u'] * origin.T[d.id]) \
                        for d in destinations])
            for d in destinations:
                T = origin.T[d.id]
                D = param['lambda'] * origin.H * self.nodes_map[d.id].E * math.exp(-param['u'] * T)
                D /= divisor
                # D = param['lambda'] * origin.H * d.E * math.exp(-param['u'] * origin.T[d.id])
                self.Ds.append(D)


    def demand_allocation(self):
        '''
            根据交通阻抗，分配需求量
        '''
        origins = self.get_origins()
        destinations = self.get_destinations()

        for o in self.get_origins():
            o.T_road = self.single_source_shortest_path(o, filter_type='metro')
            o.T_road = [t * param['t_value'] for t in o.T_road]
            o.T_metro = self.single_source_shortest_path(o, filter_type='roadway')
            o.T_metro = [t * param['t_value'] for t in o.T_metro]

        demand_metro = []
        demand_road = []
        for i, d in enumerate(self.Ds):
            o = origins[i // len(destinations)]
            t = int(destinations[i % len(destinations)].id)
            Tst_road = math.exp(-param['theta'] * o.T_road[t])
            Tst_metro = math.exp(-param['theta'] * o.T_metro[t])
            Ts_total = Tst_road + Tst_metro
            # 除了PNR以外，其它的AOZ的metro需求量全部给road
            if o.TAZ == 'AOZ' and self.mode != 'PNR':
                demand_metro.append(0)
                demand_road.append(d * (Tst_road + Tst_metro) / Ts_total)
            else:
                demand_metro.append(d * Tst_metro / Ts_total)
                demand_road.append(d * Tst_road / Ts_total)

        return demand_metro, demand_road


    # 均衡模型
    def UE(self):
        graph = []
        edges_except_metro = [e for e in self.edges if e.type != 'metro']
        for source in self.adj_table:
            graph.append((str(source), [str(e.target) for e in self.adj_table[source] if e.type != 'metro']))
        capacity = [e.capacity for e in edges_except_metro]
        free_time = [e.cost for e in edges_except_metro]
        origins = [str(n.id) for n in self.get_origins()]
        destinations = [str(n.id) for n in self.get_destinations()]
        # 分配需求量
        self.demand_metro, self.demand_road = self.demand_allocation()

        print('============需求量分配结果===============')
        print('总需求量：', self.Ds)
        print('地铁需求量：', self.demand_metro)
        print('道路需求量求量：', self.demand_road)
        print('=======================================')
        # print('============需求比例===============')
        # print((self.Ds[0] + self.Ds[1]) / sum(self.Ds))
        # print((self.demand_metro[2] + self.demand_metro[3]) / 
        #             (self.demand_metro[2] + self.demand_metro[3] + self.demand_road[2] + self.demand_road[3]))
        # print('=======================================')

        # print(graph)
        # print(capacity)
        # print(free_time)
        # print(origins)
        # print(destinations)
        # print(self.demand_road)

        # Initialize the model by data
        mod = TrafficFlowModel(graph, origins, destinations, self.demand_road, free_time, capacity)
        # Change the accuracy of solution if necessary
        mod._conv_accuracy = 1e-6
        # Set the precision of display, which influences only the digit of numerical component in arrays
        mod.set_disp_precision(4)
        # Solve the model by Frank-Wolfe Algorithm
        mod.solve()
        mod.report()

        # result为UE模型的迭代结果，是一个元组的列表
        # 元组的结构为（id, group, time, path）
        result = mod.UE_result()
        group_min_T = {}
        for _, group_id, time, _ in result:
            if group_id not in group_min_T:
                group_min_T[group_id] = time
            else:
                group_min_T[group_id] = min(group_min_T[group_id], time)

        g = iter(group_min_T.values())
        for i in range(len(self.nodes)):
            if self.nodes[i].type == 'origin':
                for d in self.get_destinations():
                    self.nodes[i].T[d.id] = next(g)

        # 更新UE后的v
        # Generate report to console
        # mod.report()
        # Return the solution if necessary
        # mod._formatted_solution()

        flow = mod.get_link_flow()
        # print('==================Link flow==================')
        # print(flow)
        j = 0
        for i in range(len(self.edges)):
            if self.edges[i].type != 'metro':
                self.edges[i].flow = flow[j]
                j += 1
        # print('==================Path time==================')
        link_time = mod.get_link_time()
        # print(link_time)
        j = 0
        for i in range(len(self.edges)):
            if self.edges[i].type != 'metro':
                self.edges[i].link_time = link_time[j]
                j += 1
            

    # 初始化T的值
    def init_T(self):
        for o in self.get_origins():
            o.H = o.area * o.density
            T_metro = self.single_source_shortest_path(o, filter_type='roadway')
            T_auto = self.single_source_shortest_path(o, filter_type='metro')
            # print(o.id, ' to 9 using metro:', T_metro[9])
            # print(o.id, ' to 11 using metro:', T_metro[11])
            # print(o.id, ' to 9 using auto:', T_auto[9])
            # print(o.id, ' to 11 using auto:', T_auto[11])
            T_totle = []
            for i in range(len(T_metro)):
                if T_metro[i] < INF and T_auto[i] < INF:
                    T_totle.append(math.log(math.exp((T_metro[i] + T_auto[i]) * param['t_value'])))
                else:
                    T_totle.append(INF)
            o.T = T_totle
            # o.T = [t * param['t_value'] for t in o.T]



    # 运行模型
    def run(self, max_epoch):
        self.init_T()

         # T需要重新算
        # AOZ_node.T[9] = 1.31
        # AOZ_node.T[11] = 1.16
        # TSA_node.T[9] = 1.26
        # TSA_node.T[11] = 1.26
        
        # AOZ_node = self.get_AOZ_node()
        # TSA_node = self.get_TSA_node()
        # print(AOZ_node.T[9])
        # print(AOZ_node.T[11])
        # print(TSA_node.T[9])
        # print(TSA_node.T[11])

        self.hist_VKT = []
        self.hist_MP = []
        self.hist_VHD = []
        self.hist_demand_1_9 = []
        self.hist_demand_1_11 = []
        self.hist_demand_2_9 = []
        self.hist_demand_2_11 = []
        for _ in range(max_epoch):
            print('=================step %d================' % (_))
            self.RRM()
            self.TDM()
            self.UE()
            # print('==================D[i, j]==================')
            Demand_ratio = []
            origin_num = len(self.get_origins())
            destination_num = len(self.get_destinations())
            D_sum = sum(self.Ds)
            i = 0
            for _ in range(origin_num):
                origin_sum = 0
                for _ in range(destination_num):
                    origin_sum += self.Ds[i]
                    i += 1
                Demand_ratio.append(origin_sum / D_sum)
            

            self.hist_VKT.append(VKTTarget(self).target_fn())
            self.hist_MP.append(MPTarget(self).target_fn())
            self.hist_VHD.append(VHDTarget(self).target_fn())
            self.hist_demand_1_9.append(self.Ds[0])
            self.hist_demand_1_11.append(self.Ds[1])
            self.hist_demand_2_9.append(self.Ds[2])
            self.hist_demand_2_11.append(self.Ds[3])

        # print('==================H==================')
        # origins = self.get_origins()
        # for o in origins:
        #     print('id = %d, H = %d' % (o.id,o.H))


    # 通过节点类型寻找节点，返回一个节点的列表
    def search_nodes_by_type(self, type):
        return [n for n in self.nodes if n.type == type]

    # 得到所有的起点
    def get_origins(self):
        return self.search_nodes_by_type('origin')

    # 得到所有的终点
    def get_destinations(self):
        return self.search_nodes_by_type('destination')

    def get_AOZ_node(self):
        return [n for n in self.nodes if n.TAZ == 'AOZ'][0]
    
    def get_TSA_node(self):
        return [n for n in self.nodes if n.TAZ == 'TSA'][0]

    # 初始化h值
    def init_h(self, origin_node):
        AOZ_node = self.get_AOZ_node()
        TSA_node = self.get_TSA_node()
        h = param['alpha'] * AOZ_node.H * math.exp(param['beta'] * TSA_node.A) / \
            (math.exp(param['beta'] * TSA_node.A) + math.exp(param['beta'] * AOZ_node.A))

        
        Du = TSA_node.LT * TSA_node.De + (TSA_node.area - TSA_node.LT) * TSA_node.density
        bound = Du - origin_node.H
        h = bound if h > bound else h
        return h

    def single_source_shortest_path(self, source, filter_type=None):
        '''
            狄杰斯特拉算法，求单源最短路径
            节点的id默认的是1到N
        '''
        # INF表示无穷大
        INF = 1000000000000
        n = len(self.nodes)
        D = [INF] * (n + 1)
        visited = [False] * (n + 1)
        D[source.id] = 0
        visited[source.id] = True
        new_node = source.id
        for _ in range(n - 1):
            # 遍历相邻new node的点，松弛节点的当前最短路径
            for adj_e in self.adj_table[new_node]:
                if filter_type is not None and adj_e.type == filter_type:
                    continue
                if D[adj_e.target] > D[new_node] + adj_e.cost:
                    D[adj_e.target] = D[new_node] + adj_e.cost
            # 寻找下一个最短路径
            min_d = INF
            for i in range(1, n + 1):
                if not visited[i] and D[i] < min_d:
                    min_d = D[i]
                    new_node = i
            visited[new_node] = True

        return [round(d, 3) for d in D]


class BaseTarget(object):
    '''
        base class of target function
    '''
    def target_fn(self):
        pass


class VKTTarget(BaseTarget):
    '''

    '''
    def __init__(self, graph):
        self.graph = graph
    
    def target_fn(self):
        return sum([e.flow * e.length for e in self.graph.edges if e.type == 'roadway'])


class MPTarget(BaseTarget):
    '''

    '''
    def __init__(self, graph):
        self.graph = graph
    
    def target_fn(self):
        return sum(self.graph.demand_metro)

class VHDTarget(BaseTarget):
    '''

    '''
    def __init__(self, graph):
        self.graph = graph
    
    def target_fn(self):
        T_actual = sum([e.flow * e.link_time for e in self.graph.edges])
        T_free = 0
        dests_id = [node.id for node in self.graph.get_destinations()]
        i = 0
        for o in self.graph.get_origins():
            shortest_time = self.ideal_shortest_time(o)
            for d_id in dests_id:
                T_free += shortest_time[d_id] * self.graph.demand_road[i]
                i += 1
        print('actual: ', T_actual)
        print('free: ', T_free)
        return T_actual - T_free

    def ideal_shortest_time(self, source, filter_type=None):
        '''
            狄杰斯特拉算法，求单源最短路径
            节点的id默认的是1到N
        '''
        # INF表示无穷大
        INF = 1000000000000
        n = len(self.graph.nodes)
        D = [INF] * (n + 1)
        visited = [False] * (n + 1)
        D[source.id] = 0
        visited[source.id] = True
        new_node = source.id
        for _ in range(n - 1):
            # 遍历相邻new node的点，松弛节点的当前最短路径
            for adj_e in self.graph.adj_table[new_node]:
                if filter_type is not None and adj_e.type == filter_type:
                    continue
                if D[adj_e.target] > D[new_node] + adj_e.length:
                    D[adj_e.target] = D[new_node] + adj_e.length
            # 寻找下一个最短路径
            min_d = INF
            for i in range(1, n + 1):
                if not visited[i] and D[i] < min_d:
                    min_d = D[i]
                    new_node = i
            visited[new_node] = True

        return [round(d / param['speed'], 3) for d in D]


if __name__ == '__main__':
    param['cl'] = 4000
    param['cl_hat'] = 15000
    param['cp'] = 15000
    param['De1'] = 2500
    param['De2'] = 2500
    param['E9'] = 60
    param['E11'] = 60
    param['L1'] = 15
    param['L2'] = 2
    param['tl0'] = 0.15
    param['tp0'] = 0.5
    param['t_hat_operate_metro'] = 0.14
    param['t_hat_walk'] = 0.05
    param['t_hat_wait_metro'] = 0.05
    param['alpha'] = 0.9
    param['beta'] = 0.1
    param['theta'] = 2
    param['u'] = 0.9
    param['lambda'] = 0.3 # 此处有魔改，原先是0.1
    param['t_value'] = 1
    param['Lp'] = 0.03
    param['speed'] = 60

    nodes = []
    nodes.append(Node(1, type='origin', TAZ='AOZ', area=15, density=2500, L=15, D=2500))
    nodes.append(Node(2, type='origin', TAZ='TSA', area=2, density=2500, L=2, D=2500, LT=2, De=7500))
    nodes.append(Node(3, type='ordinary'))
    nodes.append(Node(4, type='ordinary'))
    nodes.append(Node(5, type='ordinary'))
    nodes.append(Node(6, type='ordinary'))
    nodes.append(Node(7, type='ordinary'))
    nodes.append(Node(8, type='ordinary'))
    nodes.append(Node(9, type='destination', E=60))
    nodes.append(Node(10, type='ordinary'))
    nodes.append(Node(11, type='destination', E=60))

    edges = []
    edges.append(Edge(source=1, target=2, type='access', capacity=INF, length=9))
    edges.append(Edge(source=1, target=3, type='roadway', capacity=4000, length=9))
    edges.append(Edge(source=1, target=4, type='roadway', capacity=4000, length=9))
    edges.append(Edge(source=2, target=3, type='roadway', capacity=4000, length=9))
    edges.append(Edge(source=2, target=9, type='metro', stations_num=4, capacity=15000, length=9))
    edges.append(Edge(source=2, target=11, type='metro', stations_num=4, capacity=15000, length=9))
    edges.append(Edge(source=3, target=5, type='roadway', capacity=4000, length=9))
    edges.append(Edge(source=3, target=6, type='roadway', capacity=4000, length=9))
    edges.append(Edge(source=4, target=5, type='roadway', capacity=4000, length=9))
    edges.append(Edge(source=4, target=10, type='roadway', capacity=4000, length=9))
    edges.append(Edge(source=5, target=7, type='roadway', capacity=4000, length=9))
    edges.append(Edge(source=6, target=7, type='roadway', capacity=4000, length=9))
    edges.append(Edge(source=6, target=8, type='roadway', capacity=4000, length=9))
    edges.append(Edge(source=7, target=9, type='roadway', capacity=4000, length=9))
    edges.append(Edge(source=7, target=11, type='roadway', capacity=4000, length=9))
    edges.append(Edge(source=8, target=9, type='roadway', capacity=4000, length=9))
    edges.append(Edge(source=10, target=11, type='roadway', capacity=4000, length=9))

    # 改为无向图模型
    # edges_extend = []
    # for e in edges:
    #     ex = copy.deepcopy(e)
    #     ex.source, ex.target = ex.target, ex.source
    #     edges_extend.append(ex)
    # edges.extend(edges_extend)


    graph = Graph(nodes, edges, mode="PNR")

    max_epoch = 15
    graph.run(max_epoch)

    print('=================VKT target=================')
    print(VKTTarget(graph).target_fn())
    print('=================MP target=================')
    print(MPTarget(graph).target_fn())
    print('=================VHD target=================')
    print(VHDTarget(graph).target_fn())

    fig, axs = plt.subplots(2, 2)
    fig.suptitle('Results')

    axs[0, 0].plot(range(1, max_epoch + 1), graph.hist_VKT, 'tab:orange')
    axs[0, 0].set_title("VKT target")
    axs[0, 0].set(xlabel='', ylabel='VKT value')

    axs[0, 1].plot(range(1, max_epoch + 1), graph.hist_MP, 'tab:green')
    axs[0, 1].set_title("MP target")
    axs[0, 1].set(xlabel='', ylabel='MP value')

    axs[1, 0].plot(range(1, max_epoch + 1), graph.hist_VHD, 'tab:red')
    axs[1, 0].set_title("VHD target")
    axs[1, 0].set(xlabel='step', ylabel='VHD value')

    axs[1, 1].plot(range(1, max_epoch + 1), graph.hist_demand_1_9, 'k', label='1 to 9')
    axs[1, 1].plot(range(1, max_epoch + 1), graph.hist_demand_1_11, 'r-.', label='1 to 11')
    axs[1, 1].plot(range(1, max_epoch + 1), graph.hist_demand_2_9, 'b--', label='2 to 9')
    axs[1, 1].plot(range(1, max_epoch + 1), graph.hist_demand_2_11, 'g:', label='2 to 11')
    axs[1, 1].set_title("OD Demands")
    axs[1, 1].set(xlabel='step', ylabel='demands')
    axs[1, 1].legend()


    plt.show()