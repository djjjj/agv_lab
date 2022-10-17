a = set()
for i in range(edge.shape[0]):
    for j in range(edge.shape[1]):
        if edge[i, j] == 0:
            a.add((j-1, i))
            break
    for j in range(edge.shape[1] - 1, -1, -1):
        if edge[i, j] == 0:
            a.add((j + 1, i))
            break
for j in range(edge.shape[1]):
    for i in range(edge.shape[0]):
        if edge[i, j] == 0:
            a.add((j, i - 1))
            break
    for i in range(edge.shape[0] - 1, -1, -1):
        if edge[i, j] == 0:
            a.add((j, i+1))
            break
            
a
