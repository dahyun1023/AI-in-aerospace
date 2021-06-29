def ids(graph, start):
    visited = []
    queue   = [start]
    stack = [start]
    t = 1
    
    while t:
        print('stack   :',stack)
        while queue:
            n = queue.pop()
            print('node    :',n)
            visited.append(n)
            print('visited :',visited)  
            if n == 'G':
                print('goal is found')
                t = 0
                break
                
        while stack:
            n = stack.pop()
            queue += graph[n]  - set(visited)
        stack = queue[:]
    return visited
    
    
ids(graph, 'S')    
