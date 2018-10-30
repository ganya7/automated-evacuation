import heapq

def dijkstra(g,source,end):
	dist = [100]*len(g)
	dist[source] = 0 #distance source vertex = 0, rest infinity
	vertex = []
	w = 0
	e = 0
	vis_d = [False]*len(g)	#initialize all vertices as unvisited and insert them in queue
	for i in range(len(g)):
		heapq.heappush(vertex,(dist[i],i))
	parent = [-1]*n 	#array to store the shortest path from source
	while vertex:	#until all vertices are visited
		wei,v = heapq.heappop(vertex)
		if vis_d[v] == False:
			vis_d[v] = True
		for i in range(len(g)):
			if graph[v][i] > 0:		#connected nodes of current vertex
				e = i 	#edge to connected node
				w = graph[v][i] 	#weight of current and connected vertex
			if dist[v] + w < dist[e]:
				dist[e] = dist[v] + w
				parent[e] = v
				heapq.heappush(vertex,(dist[e],v))
	path = []
	path.append(source)
	print("Path:")
	path = path_print(parent,end,path)
	print(path)
	return dist[end]

def path_print(parent,end,path):
	if parent[end] == -1:
		return
	path_print(parent,parent[end],path)
	path.append(end)
	return path

n = int(input("Enter number of vertex:"))
graph = []
print("Enter the weights in matrix:")
for i in range(n):
	print(i,":",end="")
	graph_temp = [int(arr_temp) for arr_temp in input().strip().split()]
	graph.append(graph_temp)
start = input("Enter start node:")
goal = input("Enter goal node:")
goal_dist = dijkstra(graph,int(start),int(goal))
