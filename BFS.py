def bfs(graph, start):
    visited = []
    queue = [start]

    while queue:
        n = queue.pop(0)

        if n not in visited:
            print('node    :',n)
            print('queue   :',queue)
            visited.append(n)
            queue +=  graph[n] - set(visited)          
            print('queuenew:',queue)
            print('visited :',visited)        
        if n == 'G':
            break
    return visited

bfs(graph, 'S')



def bfs2(graph, start):
    visited = []
    queue = [start]

    while queue:
        n = queue.pop(0)
        print('node    :',n)
        print('queue   :',queue)
        
        visited.append(n)
        queue += graph[n] - set(visited)
        print('queuenew:',queue)
        print('visited :',visited)        
        if n == 'G':
            break
    return visited

bfs(g   raph, 'S')
