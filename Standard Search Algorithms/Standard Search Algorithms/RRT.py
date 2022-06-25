# Standard Algorithm Implementation
# Sampling-based Algorithms RRT and RRT*

import matplotlib.pyplot as plt
import numpy as np
import math
from scipy import spatial

# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.parent = None  # parent node
        self.cost = 0.0  # cost

# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size

        self.start = Node(start[0], start[1])  # start node
        self.goal = Node(goal[0], goal[1])  # goal node
        self.vertices = []  # list of nodes
        self.found = False  # found flag

    def init_map(self):
        '''Intialize the map before each search
        '''
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    def dis(self, node1, node2):
        '''Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        '''
        ### YOUR CODE HERE ###
        x_coordinate, y_coordinate = node1.row, node1.col
        x_2coord, y_2coord = node2.row, node2.col
        return np.round(math.sqrt((x_2coord - x_coordinate) * (x_2coord - x_coordinate) + (y_2coord - y_coordinate) * (y_2coord - y_coordinate)), 2)

    def check_collision(self, node1, node2):
        '''Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        '''
        ### YOUR CODE HERE ###

        # Check obstacle between nodes
        # get all the points in between
        points_between = zip(np.linspace(node1.row, node2.row, dtype=int),
                             np.linspace(node1.col, node2.col, dtype=int))
        # check if any of these are obstacles
        for point in points_between:
            if self.map_array[point[0]][point[1]] == 0:
                return True
        return False

    def get_new_point(self, goal_bias):
        '''Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal instead of a random point

        return:
            point - the new point
        '''
        ### YOUR CODE HERE ###

        x_coord = np.random.randint(0, self.size_col - 1, dtype=int)
        y_coord = np.random.randint(0, self.size_row - 1, dtype=int)
        random_Point = Node(x_coord, y_coord)
        choice_List = [self.goal, random_Point]
        bias_list = [0, 1]
        bias_Value = np.random.choice(bias_list, 1, p=[goal_bias, 1 - goal_bias])
        return choice_List[bias_Value[0]]

    def get_nearest_node(self, point):
        '''Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        '''
        ### YOUR CODE HERE ###
        samples = []
        for i in self.vertices:
            samples.append([i.row, i.col])
        kdtree = spatial.KDTree(samples)
        dis, index = kdtree.query(point, k=1)
        return self.vertices[index]

    def get_neighbors(self, new_node, neighbor_size):
        '''Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - a list of neighbors that are within the neighbor distance
        '''
        ### YOUR CODE HERE ###
        samples = []
        for i in self.vertices:
            samples.append([i.row, i.col])

        kdtree = spatial.KDTree(samples)
        neigh_Index = kdtree.query_ball_point(new_node, neighbor_size)
        return neigh_Index

    def rewire(self, new_node, neighbors):
        '''Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors that are within the neighbor distance from the node

        Rewire the new node if connecting to a new neighbor node will give least cost.
        Rewire all the other neighbor nodes.
        '''
        ### YOUR CODE HERE ###
        min_cost = float("inf")

        for node in neighbors:
            if not self.check_collision(self.vertices[node], new_node):
                if new_node.cost + self.dis(new_node, self.vertices[node]) < self.vertices[node].cost:
                    self.vertices[node].parent = new_node

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker='o', color='y')
            plt.plot([node.col, node.parent.col], [node.row, node.parent.row], color='y')

        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot([cur.col, cur.parent.col], [cur.row, cur.parent.row], color='b')
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker='o', color='b')

        # Draw start and goal
        plt.plot(self.start.col, self.start.row, markersize=5, marker='o', color='g')
        plt.plot(self.goal.col, self.goal.row, markersize=5, marker='o', color='r')

        # show image
        plt.show()

    def RRT(self, n_pts=1000):
        '''RRT main search function
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        In each step, extend a new node if possible, and check if reached the goal
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###
        # get a new point and get its nearest node,
        for i in range(n_pts):
            dist = 10
            new_point = self.get_new_point(0.05)
            node_near = self.get_nearest_node([new_point.row, new_point.col])

            slope = np.arctan2(new_point.col - node_near.col, new_point.row - node_near.row)
            n_row = int(node_near.row + dist * math.cos(slope))
            n_col = int(node_near.col + dist * math.sin(slope))
            if 0 <= n_row < self.size_row - 1 and 0 <= n_col < self.size_col - 1:
                newNode = Node(n_row, n_col)
                #Check collision to add or drop it
                if not self.check_collision(newNode, node_near):
                    newNode.parent = node_near
                    newNode.cost = node_near.cost + self.dis(newNode, node_near)
                    self.vertices.append(newNode)

                #check for the goal in region
                if not self.found:
                    if self.dis(newNode, self.goal) < dist:
                        self.found = True
                        self.goal.parent = newNode
                        self.goal.cost = newNode.cost + self.dis(self.goal, newNode)
                        break

        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" % steps)
            print("The path length is %.2f" % length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()

    def RRT_star(self, n_pts=1000, neighbor_size=20):
        '''RRT* search function
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points
            neighbor_size - the neighbor distance

        In each step, extend a new node if possible, and rewire the node and its neighbors
        '''
        # Remove previous result
        self.init_map()

        ### YOUR CODE HERE ###

        # get a new point and get its nearest node,
        for i in range(n_pts):
            dist = 10
            new_point = self.get_new_point(0.005)
            node_near = self.get_nearest_node([new_point.row, new_point.col])

            slope = np.arctan2(new_point.col - node_near.col, new_point.row - node_near.row)
            n_row = int(node_near.row + dist * math.cos(slope))
            n_col = int(node_near.col + dist * math.sin(slope))

            if 0 <= n_row < self.size_row - 1 and 0 <= n_col < self.size_col - 1:

                newNode = Node(n_row, n_col)

                if not self.check_collision(newNode, node_near):
                    min_cost = float("inf")
                    neighborList = self.get_neighbors((n_row, n_col), 20)
                    for index in neighborList:
                        node = self.vertices[index]
                        if node.cost + self.dis(node, newNode) < min_cost and not self.check_collision(node, newNode):
                            min_cost = node.cost + self.dis(node, newNode)
                            newNode.parent = node
                            newNode.cost = min_cost
                    self.vertices.append(newNode)

                    self.rewire(newNode, neighborList)

                #check if reach the neighbor region of the goal if the path is not found.
                if not self.found:
                    if self.dis(newNode, self.goal) < dist:
                        self.found = True
                        self.goal.parent = newNode
                        self.goal.cost = newNode.cost + self.dis(self.goal, newNode)
        # Output
        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" % steps)
            print("The path length is %.2f" % length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
