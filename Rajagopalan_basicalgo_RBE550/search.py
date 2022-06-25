# Basic searching algorithms
import math
import copy


class Node:

    def __init__(self, coordinate, parent):
        self.coordinate = coordinate
        self.parent = parent


def dfs_neighbour_nodes(grid, coordinate):
    width = len(grid[0]) - 1
    height = len(grid) - 1
    actions = [(coordinate[0] - 1, coordinate[1]), (coordinate[0], coordinate[1] - 1),
               (coordinate[0] + 1, coordinate[1]), (coordinate[0], coordinate[1] + 1)]

    valid_cell_list = []
    for x, y in actions:
        if 0 <= x <= height and 0 <= y <= width:  #
            valid_cell_list.append((x, y))
    node_list = []
    for x, y in valid_cell_list:
        if grid[x][y] != 1:
            node_list.append((x, y))

    return node_list

def bfs_dijk_neighbour_nodes(grid, coordinate):

    width = len(grid[0]) - 1
    height = len(grid) - 1

    actions = [(coordinate[0] - 1, coordinate[1]), (coordinate[0], coordinate[1] - 1),
               (coordinate[0] + 1, coordinate[1]), (coordinate[0], coordinate[1] + 1)]

    valid_cell_list = []
    for x, y in actions:
        if 0 <= x <= height and 0 <= y <= width:
            valid_cell_list.append((x, y))
    node_list = []

    for x, y in valid_cell_list:
        if grid[x][y] != 1:
            node_list.append((x, y))
    node_list.reverse()

    return node_list


def bfs(grid, start, goal):
    """Return a path found by BFS alogirhm
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node),
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution,
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    """
    goal = tuple(goal)
    start = tuple(start)
    path = []
    steps = 0
    found = False
    node = Node(start, None)
    duplicate = []
    previous_nodes = set()
    previous_nodes.add(start)

    while True:
        if node.coordinate == goal:     #check for goal or not
            found = True

            while node.parent is not None:
                path.append(list(node.coordinate))
                node = node.parent
            path.reverse()
            break
        for x, y in bfs_dijk_neighbour_nodes(grid, node.coordinate):
            if not (x, y) in previous_nodes:
                duplicate.append(Node((x, y), node))
        if len(duplicate) == 0:
            break  #if goal reached break

        node = duplicate[0]
        duplicate = duplicate[1:]
        previous_nodes.add(node.coordinate)
    steps = len(previous_nodes)
    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        print("No path found")
    path.insert(0, list(start))
    return path, steps


def dfs(grid, start, goal):
    """Return a path found by DFS alogirhm
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node),
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution,
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    """
    goal = tuple(goal)
    start = tuple(start)
    path = []
    steps = 0
    found = False
    node = Node(start, None)
    duplicate = []
    previous_nodes = set()
    previous_nodes.add(start)

    while True:
        if node.coordinate == goal:   #check if goal or not
            found = True

            while node.parent is not None:
                path.append(list(node.coordinate))
                node = node.parent

            path.reverse()
            break       # out og loop if goal or not

        for x, y in dfs_neighbour_nodes(grid, node.coordinate):
            if not (x, y) in previous_nodes:
                duplicate.append(Node((x, y), node))

        if len(duplicate) == 0:
            break

        node = duplicate.pop()
        previous_nodes.add(node.coordinate)
        steps = len(previous_nodes)

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        print("No path found")

    path.insert(0, list(start))
    return path, steps


def dijkstra(grid, start, goal):
    """Return a path found by Dijkstra alogirhm
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node),
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution,
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> dij_path, dij_steps = dijkstra(grid, start, goal)
    It takes 10 steps to find a path using Dijkstra
    >>> dij_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    """
    goal = tuple(goal)
    start = tuple(start)
    path = []
    steps = 0
    found = False
    node = Node(start, None)
    duplicate = []
    previous_nodes = set()
    previous_nodes.add(start)
    duplicate_coordinates = []
    while True:
        if node.coordinate == goal:
            found = True

            while node.parent is not None:
                path.append(list(node.coordinate))
                node = node.parent
            path.reverse()
            break

        dj_node_list = []
        for x, y in bfs_dijk_neighbour_nodes(grid, node.coordinate):
            dis = math.sqrt((x - goal[0]) * (x - goal[0])) + math.sqrt((y - goal[1]) * (y - goal[1]))
            dj_node_list.append(((x, y), dis))

        dj_node_list.sort(key=lambda num: num[1])

        for (x, y), z in dj_node_list:
            if not (x, y) in previous_nodes:
                duplicate.append(Node((x, y), node))

        if len(duplicate) == 0:
            break

        node = duplicate[0]
        duplicate = duplicate[1:]
        duplicate_coordinates = duplicate_coordinates[1:]
        previous_nodes.add(node.coordinate)
        steps = len(previous_nodes)

    if found:
        print(f"It takes {steps} steps to find a path using Dijkstra")
    else:
        print("No path found")

    path.insert(0, list(start))
    return path, steps


def astar(grid, start, goal):
    """Return a path found by A* alogirhm
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node),
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]
    steps - Number of steps it takes to find the final solution,
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    """

    goal = tuple(goal)
    start = tuple(start)
    path = []
    steps = 0
    found = False
    node = Node(start, None)
    duplicate = []
    previous_nodes = set()
    previous_nodes.add(start)
    duplicate_coordinates = []

    while True:
        if node.coordinate == goal:
            found = True
            while node.parent is not None:
                path.append(list(node.coordinate))
                node = node.parent
            path.reverse()
            break

        node_distance = copy.copy(node)
        dis_start = 0
        while node_distance.parent is not None:
            dis_start += 1
            node_distance = node_distance.parent

        astar_node_list = []

        for x, y in bfs_dijk_neighbour_nodes(grid, node.coordinate):
            dis_goal = math.sqrt((x - goal[0]) * (x - goal[0])) + math.sqrt((y - goal[1]) * (y - goal[1]))
            dis = dis_goal + dis_start
            astar_node_list.append(((x, y), dis))

        astar_node_list.sort(key=lambda z: z[1])

        for (x, y), dis in astar_node_list:
            if not (x, y) in previous_nodes:
                duplicate.append((Node((x, y), node), dis))
                duplicate_coordinates.append(((x, y), dis))

        duplicate.sort(key=lambda z: z[1])

        if len(duplicate) == 0:
            break

        node = duplicate[0][0]
        duplicate = duplicate[1:]
        duplicate_coordinates = duplicate_coordinates[1:]
        previous_nodes.add(node.coordinate)
        steps = len(previous_nodes)
    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        print("No path found")
    path.insert(0, list(start))
    return path, steps


def neighbour_astar(grid, node):


    width = len(grid[0]) - 1
    height = len(grid) - 1
    coordinate = node.coordinate

    actions = [(coordinate[0] - 1, coordinate[1]), (coordinate[0], coordinate[1] - 1),
               (coordinate[0] + 1, coordinate[1]), (coordinate[0], coordinate[1] + 1)]
    valid_cell_list = []
                               # Append all the valid coordinates in the map regardless of obstacle
    for x, y in actions:
        if 0 <= x <= height and 0 <= y <= width:  #
            valid_cell_list.append((x, y))
    node_list = []

    for x, y in valid_cell_list:
        if grid[x][y] != 1:
            node_list.append((x, y))
    return node_list

# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples

    # Test all the functions
    testmod()