import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import math


# Class for each node in the grid
class Node:
    def __init__(self, row, col, is_obs, is_dy_obs):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.is_obs = is_obs  # obstacle?
        self.is_dy_obs = is_dy_obs  # dynamic obstacle?
        self.tag = "NEW"  # tag ("NEW", "OPEN", "CLOSED")
        self.h = math.inf  # cost to goal (NOT heuristic)
        self.k = math.inf  # best h
        self.parent = None  # parent node


class DStar:
    def __init__(self, grid, dynamic_grid, start, goal):
        # Maps
        self.grid = grid  # the pre-known grid map
        self.dynamic_grid = dynamic_grid  # the actual grid map (with dynamic obstacles)
        # Create a new grid to store nodes
        size_row = len(grid)
        size_col = len(grid[0])
        self.grid_node = [[None for i in range(size_col)] for j in range(size_row)]
        for row in range(size_row):
            for col in range(size_col):
                self.grid_node[row][col] = self.instantiate_node((row, col))

        # The start node
        self.start = self.grid_node[start[0]][start[1]]
        # The goal node
        self.goal = self.grid_node[goal[0]][goal[1]]

        # List
        self.open = set()

        # Result
        self.path = []

    def instantiate_node(self, point):
        ''' Instatiate a node given point (x, y) '''
        row, col = point
        node = Node(row, col, not self.grid[row][col],
                    not self.dynamic_grid[row][col])
        return node

    def get_k_min(self):
        '''Get the minimal k value from open list

        return:
        k - the minimal k value in the open list;
            return -1 if the open list is empty
        '''
        # Find the node with minimal k value
        node = self.min_node()
        # If the node is None / open list is empty
        if node == None:
            return -1
        # Get the minimal k value
        else:
            return node.k

    def min_node(self):
        '''Get the node with minimal k value from open list

        return:
        node - the node with minimal k value in the open list;
               return None if the open list is empty
        '''
        # If open is empty
        if len(self.open) == 0:
            return None
        # Find the minimum value
        else:
            return min(self.open, key=lambda n: n.k)

    def delete(self, node):
        ''' Remove a node from open list
            and set it to "CLOSED"
        '''
        self.open.remove(node)
        node.tag = "CLOSED"

    def get_neighbors(self, node):
        ''' Get neighbors of a node with 8 connectivity '''
        row = node.row
        col = node.col
        neighbors = []
        # All the 8 neighbors
        for i in range(-1, 2):
            for j in range(-1, 2):
                # Check range
                if row + i < 0 or row + i >= len(self.grid) or \
                        col + j < 0 or col + j >= len(self.grid[0]):
                    continue
                # Do not append the same node
                if i == 0 and j == 0:
                    continue

                neighbors.append(self.grid_node[row + i][col + j])

        return neighbors

    def cost(self, node1, node2):
        ''' Euclidean distance from one node to another

            return:
            distance - Euclidean distance from node 1 to node 2
                       math.inf if one of the node is obstacle
        '''
        # If any of the node is an obstacle
        if node1.is_obs or node2.is_obs:
            return math.inf
        # Euclidean distance
        a = node1.row - node2.row
        b = node1.col - node2.col
        return (a ** 2 + b ** 2) ** (1 / 2)

    def process_state(self):
        ''' Pop the node in the open list
            Process the node based on its state (RAISE or LOWER)
            If RAISE
                Try to decrease the h value by finding better parent from neighbors
                Propagate the cost to the neighbors
            If LOWER
                Attach the neighbor as the node's child if this gives a better cost
                Or update this neighbor's cost if it already is
        '''
        #### TODO ####
        # Pop node from open list
        node_m = self.min_node()  # Finding node with min K value
        if node_m is None:
            return -1
        k = self.get_k_min()  # Finding K value
        self.delete(node_m)  # changing tag to closed

        # Checking to see if this is a raised state
        # If node k is smaller than h (RAISE)
        if k < node_m.h:
            # Get neighbors of the node using self.get_neighbors
            for neigh in self.get_neighbors(node_m):
                if neigh.h <= k and node_m.h > neigh.h + self.cost(neigh, node_m):
                    node_m.parent = neigh
                    node_m.h = neigh.h + self.cost(neigh, node_m)
        # If node k is the same as h (LOWER)
        if k == node_m.h:
            for neigh in self.get_neighbors(node_m):
                if neigh.tag == 'NEW' or (neigh.parent == node_m and neigh.h != node_m.h + self.cost(node_m, neigh)) or (
                        neigh.parent != node_m and neigh.h > node_m.h + self.cost(node_m, neigh)):
                    neigh.parent = node_m
                    self.insert(neigh, node_m.h + self.cost(node_m, neigh))

        # Else node k is smaller than h (RAISE)
        else:
            for neigh in self.get_neighbors(node_m):
                if neigh.tag == 'NEW' or (neigh.parent == node_m and neigh.h != node_m.h + self.cost(node_m, neigh)):
                    neigh.parent = node_m
                    self.insert(neigh, node_m.h + self.cost(node_m, neigh))
                else:
                    if neigh.parent != node_m and neigh.h > node_m.h + self.cost(node_m, neigh):
                        self.insert(node_m, node_m.h)
                    else:
                        if neigh.parent != node_m and node_m.h > neigh.h + self.cost(neigh,
                                                                                 node_m) and neigh.tag == 'CLOSED' and neigh.h > k:
                            self.insert(neigh, neigh.h)

        #### TODO END ####

        return self.get_k_min()

    def repair_replan(self, node):
        ''' Replan the trajectory until
            no better path is possible or the open list is empty
        '''
        #### TODO ####
        # Call self.process_state() until it returns k_min >= h(Y) or open list is empty
        # The cost change will be propagated
        while len(self.open):
            if self.get_k_min() > node.h:
                break
            self.process_state()

        #### TODO END ####

    def modify_cost(self, obsatcle_node, neighbor):
        ''' Modify the cost from the affected node to the obstacle node and
            put it back to the open list
        '''
        #### TODO ####
        # Change the cost from the dynamic obsatcle node to the affected node
        # by setting the obstacle_node.is_obs to True (see self.cost())
        obsatcle_node.is_obs = True
        if obsatcle_node.tag == 'CLOSED':
            # Put the obsatcle node and the neighbor node back to Open list
            self.insert(obsatcle_node, neighbor.h + self.cost(obsatcle_node, neighbor))

        #### TODO END ####

        return self.get_k_min()

    def prepare_repair(self, node):
        ''' Sense the neighbors of the given node
            If any of the neighbor node is a dynamic obstacle
            the cost from the adjacent node to the dynamic obstacle node should be modified
        '''
        #### TODO ####
        # Sense the neighbors to see if they are new obstacles
        for neigh in self.get_neighbors(node):
            # If neighbor.is_dy_obs == True but neighbor.is_obs == False,
            # the neighbor is a new dynamic obstacle
            if neigh.is_dy_obs == True and neigh.is_obs == False:
                for node_neigh in self.get_neighbors(neigh):
                    # Modify the cost from this neighbor node to all this neighbor's neighbors using self.modify_cost
                    self.modify_cost(neigh, node_neigh)
        #### TODO END ####

    def insert(self, node, new_h):
        ''' Insert node in the open list

        arguments:
        node - Node to be added
        new_h - The new path cost to the goal

        Update the k value of the node based on its tag
        Append the node t othe open_list
        '''
        # Update k
        if node.tag == "NEW":
            node.k = new_h
        elif node.tag == "OPEN":
            node.k = min(node.k, new_h)
        elif node.tag == "CLOSED":
            node.k = min(node.h, new_h)
        # Update h
        node.h = new_h
        # Update tag and put the node in the open set
        node.tag = "OPEN"
        self.open.add(node)

    def run(self):
        ''' Run D* algorithm
            Perform the first search from goal to start given the pre-known grid
            Check from start to goal to see if any change happens in the grid,
            modify the cost and replan in the new map
        '''
        #### TODO ####
        # Search from goal to start with the pre-known map
        self.insert(self.goal, 0)
        # Process until open set is empty or start is reached
        while len(self.open) != 0:
            # using self.process_state()
            self.process_state()
            if self.start == 'CLOSED':
                break
        # Visualize the first path if found
        self.get_backpointer_list(self.start)
        self.draw_path(self.grid, "Path in static map")
        if self.path == []:
            print("No path is found")
            return

        # Update the path if there is any change in the map
        # Start from start to goal
        node_current = self.start
        while node_current != self.goal:
            # Check if any repair needs to be done
            # using self.prepare_repair
            self.prepare_repair(node_current)
            # Replan a path from the current node
            # using self.repair_replan
            self.repair_replan(node_current)
            # Get the new path from the current node
            self.get_backpointer_list(node_current)
            # Get the next node to continue
            node_current = node_current.parent

            # for visualizing each move and replanning
            # Visualize the path in progress
            self.draw_path(self.dynamic_grid, "Path in progress")
            print("after get back pointer", len(self.path))
            if self.path == []:
                print("No path is found")
                return

        #### TODO END ####

    def get_backpointer_list(self, node):
        ''' Keep tracing back to get the path from a given node to goal '''
        # Assume there is a path from start to goal
        cur_node = node
        self.path = [cur_node]
        while cur_node != self.goal and \
                cur_node != None and \
                not cur_node.is_obs:
            # trace back
            cur_node = cur_node.parent
            # add to path
            self.path.append(cur_node)

        # If there is not such a path
        if cur_node != self.goal:
            self.path = []

    def draw_path(self, grid, title="Path"):
        '''Visualization of the found path using matplotlib'''
        fig, ax = plt.subplots(1)
        ax.margins()

        # Draw map
        row = len(grid)  # map size
        col = len(grid[0])  # map size
        for i in range(row):
            for j in range(col):
                if not self.grid_node[i][j].is_obs: \
                        ax.add_patch(Rectangle((j - 0.5, i - 0.5), 1, 1, edgecolor='k', facecolor='w'))  # free space
                else:
                    ax.add_patch(Rectangle((j - 0.5, i - 0.5), 1, 1, edgecolor='k', facecolor='k'))  # obstacle

        # Draw path
        for node in self.path:
            row, col = node.row, node.col
            ax.add_patch(Rectangle((col - 0.5, row - 0.5), 1, 1, edgecolor='k', facecolor='b'))  # path
        if len(self.path) != 0:
            start, end = self.path[0], self.path[-1]
        else:
            start, end = self.start, self.goal
        ax.add_patch(Rectangle((start.col - 0.5, start.row - 0.5), 1, 1, edgecolor='k', facecolor='g'))  # start
        ax.add_patch(Rectangle((end.col - 0.5, end.row - 0.5), 1, 1, edgecolor='k', facecolor='r'))  # goal
        # Graph settings
        plt.title(title)
        plt.axis('scaled')
        plt.gca().invert_yaxis()
        plt.show()
