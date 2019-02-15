import heapq

def dijkstra(g,d,source,end):
	dist = [100]*len(g)
	dist[source] = 0 #distance source vertex = 0, rest infinity
	vertex = []
	w = 0
	e = 0
	vis_d = [False]*len(g)	#initialize all vertices as unvisited and insert them in queue
	
	for i in range(len(d)):
		for j in range(len(g)):
			graph[d[i]][j] = 0
			graph[j][d[i]] = 0
		
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
	# distance = 0
	# for k in range(len(path) - 1):
		# distance = distance + graph[path[k]][path[k+1]]
	# print(distance)
	print(dist[end])
	return dist[end]

def path_print(parent,end,path):
	if parent[end] == -1:
		return
	path_print(parent,parent[end],path)
	path.append(end)
	return path

n = int(input("Enter number of vertex:"))
graph = []
danger_nodes = [2]

print("Enter the weights in matrix:")
for i in range(n):
	print(i,":",end="")
	graph_temp = [int(arr_temp) for arr_temp in input().strip().split()]
	graph.append(graph_temp)
	
#start = input("Enter start node:")
goal = [0, 5]
for i in range(n):
        if(i not in goal):
                print("Start node: ",i)
                path_selection = [100]*len(goal)
                for j in range(len(goal)):
                        if int(goal[j]) != i and (i not in danger_nodes):
                                path_selection[j] = dijkstra(graph,danger_nodes,int(i),int(goal[j]))
                min_dist = path_selection[0]
                final_goal = 0
                for j in range(1,len(path_selection)-1):
                        if path_selection[j] < min_dist:
                                min_dist = path_selection[j]
                                final_goal = j
                                
                print("final path calculated from node:")
                goal_ = dijkstra(graph,danger_nodes,int(i),int(final_goal))
