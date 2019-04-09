import serial
import time
import cv2
import threading
import time


import Inference as inf

import heapq


class ThreadingExample(object):
    """ Threading example class
    The run() method will be started and it will run in the background
    until the application exits.
    """

    def __init__(self, interval=1):
        """ Constructor
        :type interval: int
        :param interval: Check interval, in seconds
        """
        self.interval = interval

        thread = threading.Thread(target=self.run, args=())
        thread.daemon = True                            # Daemonize thread
        thread.start()                                  # Start the execution

    def run(self):
        """ Method that runs forever """
        while True:
            run_arduino()
            time.sleep(self.interval)





def dijkstra(g, dngr, source, end):
    dist = [100] * len(g)
    dist[source] = 0  # distance source vertex = 0, rest infinity
    vertex = []
    w = 0
    e = 0
    vis_d = [False] * len(g)  # initialize all vertices as unvisited and insert them in queue

    for i in range(len(dngr)):
        for j in range(len(g)):
            graph[dngr[i]][j] = 0
            graph[j][dngr[i]] = 0

    for i in range(len(g)):
        heapq.heappush(vertex, (dist[i], i))
    parent = [-1] * n  # array to store the shortest path from source
    while vertex:  # until all vertices are visited
        wei, v = heapq.heappop(vertex)
        if vis_d[v] == False:
            vis_d[v] = True
        for i in range(len(g)):
            if graph[v][i] > 0:  # connected nodes of current vertex
                e = i  # edge to connected node
                w = graph[v][i]  # weight of current and connected vertex
                if dist[v] + w < dist[e]:
                    dist[e] = dist[v] + w
                    parent[e] = v
                    heapq.heappush(vertex, (dist[e], v))
    path = []
    path.append(source)
    print("Path:")
    path = path_print(parent, end, path)
    print(path)
    distance = 0
    for k in range(len(path) - 1):
        distance = distance + graph[path[k]][path[k + 1]]
    print("Shortest distance: ",distance)
    return dist[end]


def path_print(parent, end, path):
    if parent[end] == -1:
        return
    path_print(parent, parent[end], path)
    path.append(end)
    return path


def capture_image():
    camera = cv2.VideoCapture(0)
    while True:
        return_value, image = camera.read()
        cv2.imshow('Our Face Extractor', image)
        cv2.imwrite('crowd' + '.jpg', image)
        if cv2.waitKey(1) == 13:  # 13 is the Enter Key
            break
    camera.release()


def run_arduino():
    global danger_nodes
    # arduino = serial.Serial("COM3",9600)
    for i in range(1, 2):
        # data = arduino.readline()
        # data = data.decode('utf-8')
        # data = ['Node:1', 'Node:2']
        data = ['Node:1']
        dnode = [s[5:] for s in data if 'Node:' in s]
        dnode = list(set(map(int, dnode)))
        # print(data)
        danger_nodes.extend(dnode)
        danger_nodes = list(set(danger_nodes))
        # print("danger: ", danger_nodes)
        time.sleep(0.5)


n = None
graph = list()
danger_nodes = list()
goal = None


def main_input():
    global n,goal,graph
    n = int(input("Enter number of vertex:"))
    graph = [0] * n
    for i in range(n):
        graph[i] = [0] * n
    print("Enter the weights in matrix:")
    for i in range(n-1):
        graph[i][i] = 0
        for j in range(i+1,n):
            print("Weight between edge ",i," and ",j,": ")
            graph[i][j] = int(input())
            graph[j][i] = graph[i][j]
    goal = input("Enter goal node:")


def main():
    # danger_nodes=[3]
    global n, goal
    for i in range(n):
        if int(goal) != i and (i not in danger_nodes):
            print("Start node: ", i)
            goal_ = dijkstra(graph, danger_nodes, int(i), int(goal))


if __name__ == '__main__':
    example = ThreadingExample()
    # time.sleep(30)
    # global danger_nodes
    # capture_image()
    dnode = inf.main()
    danger_nodes.extend(dnode)
    danger_nodes = list(set(danger_nodes))
    print("Danger nodes: ",danger_nodes)    #danger nodes from congestion
    main_input()
    main()
    print("Danger nodes: ",danger_nodes)    #danger nodes from arduino
    # run_arduino()
    # main()


# 6
# 0 7 9 0 0 14
# 7 0 10 15 0 0
# 9 10 0 11 0 2
# 0 15 11 0 6 0
# 0 0 0 6 0 9
# 14 0 2 0 9 0
# 4
# 0



# 4
# 0 1 2 0
# 1 0 0 10
# 2 0 0 5
# 0 10 5 0
# 3
# 0

# 6
# 0 4 3 2 0 0
# 4 0 0 2 2 0
# 3 0 0 2 0 4
# 2 2 2 0 1 0
# 0 2 0 1 0 1
# 0 0 4 0 1 0
# 4
# 0

# 4 3 2 0 0
# 0 2 2 0
# 2 0 4
# 1 0
# 1