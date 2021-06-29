graph = { 'I' : [ 'H' ],
          'H' : [ 'I', 'P', 'O' ],
          'A' : [ 'C', 'O' ],
          'S' : [ 'D', 'P' ],
          'C' : [ 'D', 'A' ],
          'D' : [ 'S', 'C' ],
          'P' : [ 'S', 'O' , 'H' ],
          'O' : [ 'P', 'A' , 'H' ]}

class Graph:
    def __init__(self, graph = {}):
        self.__graph = graph
    
    def edges(self):
        return [(node, neighbor) 
                 for node in self.__graph 
                 for neighbor in self.__graph[node]]

    def nodes(self):
        return list(self.__graph.keys())

    def isolated_nodes(self):
        return [node for node in self.__graph if not self.__graph[node]]
    
    def add_node(self, node):
        if node not in self.__graph:
            self.__graph[node] = []

    def add_edge(self, node1, node2):
        if node1 not in self.__graph:
            self.add_node(node1)
        if node2 not in self.__graph:
            self.add_node(node2)

        self.__graph[node1].append(node2)
        self.__graph[node2].append(node1)

    def all_paths(self, node1, node2, path = []):
        path = path + [node1]
        
        if node1 not in self.__graph:
            return []

        if node1 == node2:
            return [path]

        paths = []

        for node in self.__graph[node1]:
            if node not in path:

                subpaths = self.all_paths(node, node2, path)

                for subpath in subpaths:
                    paths.append(subpath)

        return paths

    def shortest_path(self, node1, node2):
        return sorted(self.all_paths(node1, node2), key = len)[0]

g = Graph(graph)

print("The paths from 'D' to 'I':")
print(g.all_paths('D','I'))
print("The shortest path: ", g.shortest_path('D','I'))

class Graph:
    
    def __init__(self, graph_dict=None, directed=True):
        self.graph_dict = graph_dict or {}
        self.directed = directed
        if not directed:
            self.make_undirected()
    
    def make_undirected(self):
        for a in list(self.graph_dict.keys()):
            for (b, dist) in self.graph_dict[a].items():
                self.graph_dict.setdefault(b, {})[a] = dist
    
    def connect(self, A, B, distance=1):
        self.graph_dict.setdefault(A, {})[B] = distance
        if not self.directed:
            self.graph_dict.setdefault(B, {})[A] = distance
    
    def get(self, a, b=None):
        links = self.graph_dict.setdefault(a, {})
        if b is None:
            return links
        else:
            return links.get(b)
    
    def nodes(self):
        s1 = set([k for k in self.graph_dict.keys()])
        s2 = set([k2 for v in self.graph_dict.values() for k2, v2 in v.items()])
        nodes = s1.union(s2)
        return list(nodes)

class Node:
    
    def __init__(self, name:str, parent:str):
        self.name = name
        self.parent = parent
        self.g = 0 
        self.h = 0 
        self.f = 0 

    def __eq__(self, other):
        return self.name == other.name

    def __lt__(self, other):
         return self.f < other.f
 
    def __repr__(self):
        return ('({0},{1})'.format(self.name, self.f))
    
    
def astar_search(graph, heuristics, start, end):
    
    open = []
    closed = []
    start_node = Node(start, None)
    goal_node = Node(end, None)
    open.append(start_node)
    
    while len(open) > 0:
        open.sort()
        current_node = open.pop(0)
        closed.append(current_node)
        
        if current_node == goal_node:
            path = []
            while current_node != start_node:
                path.append(current_node.name + ': ' + str(current_node.g))
                current_node = current_node.parent
            path.append(start_node.name + ': ' + str(start_node.g))
            return path[::-1]
        neighbors = graph.get(current_node.name)
        for key, value in neighbors.items():
            neighbor = Node(key, current_node)
            if(neighbor in closed):
                continue
            neighbor.g = current_node.g + graph.get(current_node.name, neighbor.name)
            neighbor.h = heuristics.get(neighbor.name)
            neighbor.f = neighbor.g + neighbor.h
            if(add_to_open(open, neighbor) == True):
                open.append(neighbor)
    return None

def add_to_open(open, neighbor):
    for node in open:
        if (neighbor == node and neighbor.f > node.f):
            return False
    return True

def main():
    
    graph = Graph()
    
    graph.connect('대전', '세종', 21.3)
    graph.connect('대전', '청주', 39.1)
    graph.connect('청주', '안성', 58.7)
    graph.connect('세종', '평택', 75.1)
    graph.connect('안성', '오산', 29.7)
    graph.connect('평택', '오산', 20)
    graph.connect('오산', '화성', 28.9)
    graph.connect('평택', '화성', 45)
    graph.connect('화성', '인천', 37.8)
    
    graph.make_undirected()
    
    heuristics = {}
    heuristics['대전'] = 137
    heuristics['세종'] = 121
    heuristics['청주'] = 114
    heuristics['안성'] = 71.2
    heuristics['평택'] = 62.9
    heuristics['오산'] = 47.3
    heuristics['화성'] = 30.6
    heuristics['인천'] = 0
    
    path = astar_search(graph, heuristics, '대전', '인천')
    print(path)
    print()
    
if __name__ == "__main__": main()
  
  
  
import numpy as np
import pandas as pd
harr = [137, 121, 114, 71.2, 62.9, 47.3, 30.6, 0]
df = pd.DataFrame({'city':['대전','세종', '청주', '안성', '평택', '오산', '화성', '인천'],
                   'heuristic cost-to-go':harr})
print(df)

garr1 = np.zeros((8,8))
garr1[0,1] = 21.3
garr1[0,2] = 39.1
garr1[2,3] = 58.7
garr1[1,4] = 75.1
garr1[3,5] = 29.7
garr1[4,5] = 20
garr1[5,6] = 28.9
garr1[4,6] = 45
garr1[6,7] = 37.8

garr = garr1 + garr1.T
print(garr)

df.iloc[1,:]

def f(A,B):
    fs = np.array(g_score) + np.array(h_score)
    return fs

g_score = garr1[0,0] 
h_score = harr[0]
f_score = f(g_score, h_score)

A = pd.DataFrame({'path':[df.iloc[0,0]],
                  'g':[g_score],
                  'h':[h_score],
                  'f':[f_score]
                 })
print(A)

connect = np.where(garr1[0,:] > 0)
print(connect)
connect

g_score = [garr1[0,1] , garr1[0,2]]
h_score = [harr[1] , harr[2]]
f_score = f(g_score, h_score)

A = pd.DataFrame({'path':[df.iloc[0,1], df.iloc[0,2]],
                  'g':[g_score],
                  'h':[h_score],
                  'f':[f_score]
                 })
print(A)
