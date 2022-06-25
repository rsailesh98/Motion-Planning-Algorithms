# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
from scipy import spatial


# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size

        self.samples = []  # list of sampled points
        self.graph = nx.Graph()  # constructed graph
        self.path = []  # list of nodes of the found path
        self.k_near = 20
        self.kdtree = None
        self.sampling_method = "random"

    def check_collision(self, p1, p2):
        '''Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        '''
        line_pts = zip(np.linspace(p1[0], p2[0], dtype=int), np.linspace(p1[1], p2[1], dtype=int))

        for pt in line_pts:
            if self.map_array[pt[0]][pt[1]] == 0:
                return True

        return False

    def dis(self, point1, point2):
        '''Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        '''
        ### YOUR CODE HERE ###
        euc_dist = np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
        return euc_dist

    def uniform_sample(self, n_pts):
        '''Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()
        # Generating uniform points
        row_new = int(np.sqrt(n_pts * self.size_row / self.size_col))
        coloumn_samp = int(n_pts / row_new)
        row_size = np.linspace(0, self.size_row - 1, row_new, dtype=int)
        sample_col = np.linspace(0, self.size_col - 1, coloumn_samp, dtype=int)
        p_row, p_col = np.meshgrid(row_size, sample_col)
        p_row = p_row.flatten()
        p_col = p_col.flatten()

        # Check obstacle
        for row, col in zip(p_row, p_col):
            if self.map_array[row][col] == 1:
                self.samples.append((row, col))

        return 0

    def random_sample(self, n_pts):
        '''Use random sampling and store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()
        row_size = np.random.randint(0, self.size_row - 1, n_pts, dtype=int)
        sample_col = np.random.randint(0, self.size_col - 1, n_pts, dtype=int)
        for row, col in zip(row_size, sample_col):
            if self.map_array[row][col] == 1:
                self.samples.append((row, col))

    def gaussian_sample(self, n_pts):
        '''Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()

        row_size = np.random.randint(0, self.size_row - 1, n_pts, dtype=int)
        sample_col = np.random.randint(0, self.size_col - 1, n_pts, dtype=int)
        row_gaussian = row_size + np.random.normal(0.0, 10, n_pts).astype(int)
        coloumn_gaussian = sample_col + np.random.normal(0.0, 10, n_pts).astype(int)
        for row1, col1, row2, col2 in zip(row_size, sample_col, row_gaussian, coloumn_gaussian):
            if not (0 <= row2 < self.size_row) or not (0 <= col2 < self.size_col):
                continue
            # check for obscatle and free space
            if self.map_array[row1][col1] == 1 and self.map_array[row2][col2] == 0:
                self.samples.append((row1, col1))
            elif self.map_array[row1][col1] == 0 and self.map_array[row2][col2] == 1:
                self.samples.append((row2, col2))

    def bridge_sample(self, n_pts):
        '''Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valide points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        '''
        # Initialize graph
        self.graph.clear()
        row_size = np.random.randint(0, self.size_row - 1, n_pts, dtype=int)
        sample_col = np.random.randint(0, self.size_col - 1, n_pts, dtype=int)

        for row1, col1 in zip(row_size, sample_col):
            # Remove the points outside the Maps and check points lies on collision zone
            if 0 <= col1 < self.size_col and 0 <= row1 < self.size_row:
                if self.map_array[row1][col1] == 0:
                    row_gaussian = int(np.random.normal(row1, 30, 1))
                    coloumn_gaussian = int(np.random.normal(col1, 30, 1))
                    if 0 <= coloumn_gaussian < self.size_col and 0 <= row_gaussian < self.size_row:
                        if self.map_array[row_gaussian][coloumn_gaussian] == 0:
                            # check for free space point -Mid point
                            mid_row, mid_col = int(0.5 * (row1 + row_gaussian)), int(0.5 * (col1 + coloumn_gaussian))
                            if self.map_array[mid_row][mid_col] == 1 and (mid_row, mid_col) not in self.samples:
                                self.samples.append((mid_row, mid_col))

    def draw_map(self):
        '''Visualization of the result
        '''
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict(zip(range(len(self.samples)), node_pos))
        pos['start'] = (self.samples[-2][1], self.samples[-2][0])
        pos['goal'] = (self.samples[-1][1], self.samples[-1][0])

        # draw constructed graph
        nx.draw(self.graph, pos, node_size=3, node_color='y', edge_color='y', ax=ax)

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=self.path, node_size=8, node_color='b')
            nx.draw_networkx_edges(self.graph, pos=pos, edgelist=final_path_edge, width=2, edge_color='b')

        # draw start and goal
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['start'], node_size=12, node_color='g')
        nx.draw_networkx_nodes(self.graph, pos=pos, nodelist=['goal'], node_size=12, node_color='r')

        # show image
        plt.axis('on')
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()

    def add_vertices_pairs_edge(self, pairs):
        '''Add pairs of vertices to graph as weighted edge
        arguments:
            pairs - pairs of vertices of the graph

        check collision, compute weight and add valide edges to self.graph
        '''
        for pair in pairs:
            if pair[0] == "start":
                point1 = self.samples[-2]
            elif pair[0] == "goal":
                point1 = self.samples[-1]
            else:
                point1 = self.samples[pair[0]]
            point2 = self.samples[pair[1]]

            if not self.check_collision(point1, point2):
                d = self.dis(point1, point2)
                edge = [(pair[0], pair[1], d)]
                self.graph.add_weighted_edges_from(edge)

    def sample(self, n_pts=1000, sampling_method="random"):
        '''Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        '''
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []
        self.sampling_method = sampling_method
        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "random":
            self.random_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)

        # connect to nearest kd
        pairs = []
        self.kdtree = spatial.cKDTree(list(self.samples))
        pairs_list = self.kdtree.query_pairs(self.k_near)
        for pair in pairs_list:
            point1 = self.samples[pair[0]]
            point2 = self.samples[pair[1]]
            if self.check_collision(point1, point2) == False:
                pairs.append((pair[0], pair[1], self.dis(point1, point2)))

        # p_id_list = [i for i in range(len(self.samples))]
        self.graph.add_nodes_from(range(len(self.samples)))
        # self.graph.add_nodes_from(p_id_list)
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" % (n_nodes, n_edges))

    def search(self, start, goal):
        '''Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal node, edges of them and their nearest neighbors
        to graph for self.graph to search for a path.
        '''
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(['start', 'goal'])
        ### YOUR CODE HERE ###
        tree = spatial.KDTree(self.samples)
        if self.sampling_method == "uniform":
            neighbors = tree.query_ball_point(start, 20)
        else:
            neighbors = tree.query_ball_point(start, 40)


        start_pairs = []
        goal_pairs = []
        for start_neighbor in neighbors:
            point1 = start
            point2 = self.samples[start_neighbor]
            if self.check_collision(point1, point2) == False and 'start' != start_neighbor:
                start_pairs.append(('start', start_neighbor, self.dis(point1, point2)))

        if self.sampling_method == "uniform":
            g_neighbors = tree.query_ball_point(goal, 20)
        else:
            g_neighbors = tree.query_ball_point(goal, 100)
        for goal_neighbor in g_neighbors:
            pt1 = goal
            pt2 = self.samples[goal_neighbor]
            if self.check_collision(pt1, pt2) == False and 'goal' != goal_neighbor:
                goal_pairs.append(('goal', goal_neighbor, self.dis(pt1, pt2)))

        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)

        # Search using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(self.graph, 'start', 'goal')
            path_length = nx.algorithms.shortest_paths.weighted.dijkstra_path_length(self.graph, 'start', 'goal')
            print("The path length is %.2f" % path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")

        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(['start', 'goal'])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)
