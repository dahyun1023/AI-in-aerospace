graph = {0: set([1, 2, 3]),
        1: set([4, 5]),
        2: set([4]),
        3: set([5]),
        4: set([6]),
        5: set([7, 8]),
        6: set([7]),
        7: set([7]),
        8: set([7])
}
root_node = 0
goal_node = 7

def DFS(graph, root, goal, visit):
    visited = []
    result = []
    stack = [(root,[root])]
    while stack:
        n,path= stack.pop()
        if n == goal:
            result.append(path)
        break
    else:
        for m in graph[n] - set(path):

            if m not in visited:
                if visit==True:
                    visited.append(m)
                    stack.append((m, path + [m]))
                return result

print(DFS(graph, root_node, goal_node, True))
print(DFS(graph, root_node, goal_node, False))


def dfs2(graph, start):
    visited = []
    stack   = [start]

    while stack:
        n = stack.pop()
        
        if n not in visited:
            print('node    :',n)
            print('stack   :',stack)
            
            visited.append(n)
            stack += graph[n] - set(visited)
            print('stacknew:',stack)
        print('visited :',visited)   
        if n == 'G':
            break
    return visited

dfs(graph, 'S')
