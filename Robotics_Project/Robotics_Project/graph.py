import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation
import numpy as np

from Robotics_Project.utility_robo import swap_dir_array
class Node_graph:
    def __init__(self, id):
        self.id = id
        self.position = (0, 0)
        
        self.neighbors = {
            'N': ['X', 0],  # Nord  [id, distance]
            'S': ['X', 0],  # Sud
            'E': ['X', 0],  # Est
            'O': ['X', 0]   # Ovest
        }

    def set_neighbor(self, dir, id_neighbor):
        if dir in self.neighbors:
            self.neighbors[dir] = id_neighbor
    
    def print_node_info(self):
        print(f"Node {self.id} at position {self.position}")

    def get_neighbor(self, dir):
        return self.neighbors[dir]

    def get_id(self):
        return self.id

    def set_position(self, x, y):
        self.position = (x, y)

    def get_position(self):
        return self.position

    def get_neighbor(self, dir):
        return self.neighbors[dir]
    
    # return true if all the neighbors are not "?"
    def is_complete(self):
        return all([id != '?' for id, _ in self.neighbors.values()])
    

class Graph:
    def __init__(self):
        self.nodes = {}
        self.adjacency_mat = {}
        self.unexplored = []
        self.G = nx.Graph()
        self.positions = {}
        self.not_info = []
        self.fig, self.ax = plt.subplots()

    def print_nodes(self):
        for id, node in self.nodes.items():
            node.print_node_info()
    
    def get_node(self, id):
        return self.nodes[id]

    def add_node(self, id, x, y):
        node = Node_graph(id)
        node.set_position(x, y)
        self.nodes[id] = node
        self.adjacency_mat[id] = {}
        self.positions[id] = (x, y)
        self.G.add_node(id, pos=(x, y))

    # positions = [N S E O] with 1 if there is a node in that direction
    def node_information(self, id, positions):
        # for each direction check if ther is a 1 and not a neighbor set the neighbor to '?' and add to unexplored if not already in
        for i, pos in enumerate(positions):
            if pos == 1:
                dir = ['N', 'S', 'E', 'O'][i]
                if self.nodes[id].get_neighbor(dir)[0] == 'X':
                    self.add_unexplored(id, dir)
                    self.nodes[id].set_neighbor(dir, ('?', 0))
        
        # CASE: exist a node in +-1 direction but the node in opposite direction have an X so the edge doesn't exist and we have a wall
        # since the wall exist the two nodes are not connected so we need to remove the neighbor from the new node
        for i, pos in enumerate(positions):
            if pos == 1:
                dir = ['N', 'S', 'E', 'O'][i]
                pos_dist = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'O': (-1, 0)}[dir]
                x, y = self.nodes[id].get_position()
                pos_dist = (pos_dist[0] * 1, pos_dist[1] * 1)
                x += pos_dist[0]
                y += pos_dist[1]
                # search in all nodes if there is a node with the same position
                for id_node, node in self.nodes.items():
                    x_node, y_node = node.get_position()
                    x_node, y_node = node.get_position()
                    opposite_dir = {'N': 'S', 'S': 'N', 'E': 'O', 'O': 'E'}[dir]
                    if abs(x_node - x) < 0.1 and abs(y_node - y) < 0.1 and node.get_neighbor(opposite_dir)[0] == 'X':
                        print(f"Node {id} and {id_node} are not connected")
                        self.nodes[id].set_neighbor(dir, ('X', 0))
                        # remove the unexplored direction
                        if (id, dir) in self.unexplored:
                            self.unexplored.remove((id, dir))



        # remove nodde from not_info
        if id in self.not_info:
            self.not_info.remove(id)
        
        self.update_plot()

    def exist_closed_node(self, prev_node, direction, distance, neighboor, threshold=0.1):
        pos_dist = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'O': (-1, 0)}[direction]
        x, y = self.nodes[prev_node].get_position()
        pos_dist = (pos_dist[0] * distance, pos_dist[1] * distance)
        x += pos_dist[0]
        y += pos_dist[1]
        # search in all nodes if there is a node with the same position
        for id, node in self.nodes.items():
            x_node, y_node = node.get_position()
            if abs(x_node - x) < threshold and abs(y_node - y) < threshold:
                # check if the node have same neighboor in form ex [0,1,0,1]
                neighboor_node = [1 if node.get_neighbor(dir)[0] != 'X' else 0 for dir in ['N', 'S', 'E', 'O']]
                if neighboor_node == neighboor:
                    return id
        return None


    def add_unexplored(self, id, dir):
        self.nodes[id].set_neighbor(dir, ('?', 0))
        self.unexplored.append((id, dir))

    def add_edge(self, id1, id2, direction, distance):

        
        if id2 not in self.nodes:
            pos_dist = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'O': (-1, 0)}[direction]
            x, y = self.nodes[id1].get_position()
            pos_dist = (pos_dist[0] * distance, pos_dist[1] * distance)
            x += pos_dist[0]
            y += pos_dist[1]
            self.add_node(id2, x, y)
            self.not_info.append((id2))

        self.adjacency_mat[id1][id2] = distance
        self.adjacency_mat[id2][id1] = distance

        opposite_direction = {'N': 'S', 'S': 'N', 'E': 'O', 'O': 'E'}[direction]
        self.nodes[id1].set_neighbor(direction, (id2, distance))
        self.nodes[id2].set_neighbor(opposite_direction, (id1, distance))

        # if id1 direction is in unexplored remove it
        if (id1, direction) in self.unexplored:
            self.unexplored.remove((id1, direction))
        if (id2, opposite_direction) in self.unexplored:
            self.unexplored.remove((id2, opposite_direction))

        # Add or update edges only
        self.G.add_edge(id1, id2)
        self.update_plot()


    def get_direction_node_to_node(self, id1, id2):
        if id1 == id2:
            return None
        
        x1, y1 = self.nodes[id1].get_position()
        x2, y2 = self.nodes[id2].get_position()
        if x1 == x2:
            if y1 < y2:
                return 'N'
            else:
                return 'S'
        if y1 == y2:
            if x1 < x2:
                return 'E'
            else:
                return 'O'
        return None

    def get_direction_unexplored(self, id, direction):
        opposite = {'N': 'S', 'S': 'N', 'E': 'O', 'O': 'E'}[direction]
        for dir, (id_neighbor, _) in self.nodes[id].neighbors.items():
            if id_neighbor == '?' and dir != opposite:
                return dir
        return None
            

    def closest_unexplored(self, id):
        # case 1 the node id is not complete, so we need to return the direction of the first unexplored neighbor and is id
        if not self.nodes[id].is_complete():
            for dir, (id_neighbor, _) in self.nodes[id].neighbors.items():
                if id_neighbor == '?':
                    return id, dir
                
        # case 2 the node id is complete, so we need to return the id of the closest unexplored neighbor
        # we iterate using queue
        # the node enter in visited in order of distance <--> the node pop out from queue is the closest
        # we are sure that if we iterate over a node and we find a neighbor with '?' is the closest because we are iterating in order of distance
        queue = [(id, 0)]
        visited = set()
        while queue:

            id, distance = queue.pop(0)
            visited.add(id)

            for dir, (id_neighbor, dist) in self.nodes[id].neighbors.items():
                if id_neighbor == 'X':
                    continue
                if id_neighbor == '?' and (id, dir) in self.unexplored:
                    return id, dir
                if id_neighbor != '?' and id_neighbor not in visited:
                    queue.append((id_neighbor, distance + dist))
            
            queue = sorted(queue, key=lambda x: x[1])

    # def update_plot(self):
    #     self.ax.clear()  # Clears the current axes
    #     node_color = ['blue' if id not in self.not_info else 'black' for id in self.nodes]
    #     node_color = ['red' if id in [n for n, _ in self.unexplored] else color for id, color in zip(self.nodes, node_color)]
    #     nx.draw(self.G, self.positions, with_labels=True, node_size=1000, node_color=node_color, font_size=10, font_weight='bold', font_color='black', edge_color='black', width=2, ax=self.ax)

    #     for id, dir in self.unexplored:
    #         x, y = self.positions[id]
    #         dx, dy = {'N': (0, 0.1), 'S': (0, -0.1), 'E': (0.1, 0), 'O': (-0.1, 0)}[dir]
    #         self.ax.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1, fc='gray', ec='gray')


    #     x = [x for x, _ in self.positions.values()]
    #     y = [y for _, y in self.positions.values()]
    #     x_diff = max(x) - min(x)
    #     y_diff = max(y) - min(y)
    #     max_diff = max(x_diff, y_diff)

    #     if max_diff == 0:
    #         max_diff = 1

    #     self.ax.set_xlim(min(x) - 0.1 * max_diff, max(x) + 0.1 * max_diff)
    #     self.ax.set_ylim(min(y) - 0.1 * max_diff, max(y) + 0.1 * max_diff)

    #     plt.draw()
    #     plt.pause(0.5)  # Pause to allow update

    def update_plot2(self):
        self.ax.clear()  # Clears the current axes

        # Plot the checkerboard
        self.plot_checkerboard()

        # Node colors
        node_color = ['blue' if id not in self.not_info else 'black' for id in self.nodes]
        node_color = ['red' if id in [n for n, _ in self.unexplored] else color for id, color in zip(self.nodes, node_color)]

        # Draw the graph
        nx.draw(self.G, self.positions, with_labels=True, node_size=1000, node_color=node_color, font_size=10, 
                font_weight='bold', font_color='black', edge_color='black', width=2, ax=self.ax)

        # Draw arrows for unexplored directions
        for id, dir in self.unexplored:
            x, y = self.positions[id]
            dx, dy = {'N': (0, 0.1), 'S': (0, -0.1), 'E': (0.1, 0), 'O': (-0.1, 0)}[dir]
            self.ax.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1, fc='gray', ec='gray')

        # Set axis limits
        x = [x for x, _ in self.positions.values()]
        y = [y for _, y in self.positions.values()]
        x_diff = max(x) - min(x)
        y_diff = max(y) - min(y)
        max_diff = max(x_diff, y_diff)

        if max_diff == 0:
            max_diff = 1

        self.ax.set_xlim(min(x) - 0.1 * max_diff, max(x) + 0.1 * max_diff)
        self.ax.set_ylim(min(y) - 0.1 * max_diff, max(y) + 0.1 * max_diff)

        plt.draw()
        plt.pause(0.5)  # Pause to allow update

    def update_plot(self):
        self.ax.clear()  # Clears the current axes

        # Plot the checkerboard
        self.plot_checkerboard()

        # Node colors
        node_color = ['blue' if id not in self.not_info else 'black' for id in self.nodes]
        node_color = ['red' if id in [n for n, _ in self.unexplored] else color for id, color in zip(self.nodes, node_color)]

        # Draw the graph
        nx.draw(self.G, self.positions, with_labels=True, node_size=1000, node_color=node_color, font_size=10, 
                font_weight='bold', font_color='black', edge_color='black', width=2, ax=self.ax)

        # Draw arrows for unexplored directions
        for id, dir in self.unexplored:
            x, y = self.positions[id]
            dx, dy = {'N': (0, 0.1), 'S': (0, -0.1), 'E': (0.1, 0), 'O': (-0.1, 0)}[dir]
            self.ax.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1, fc='gray', ec='gray')

        # Draw walls where there are no neighbors
        for node in self.nodes:
            x, y = self.positions[node]
            neighbors = list(self.G.neighbors(node))
            question_neighbors = [dir for dir, (id_neighbor, _) in self.nodes[node].neighbors.items() if id_neighbor == '?']

            #print(f'Node {node} has neighbors {neighbors} and question neighbors {question_neighbors}')

            n_up = False
            n_down = False
            n_right = False
            n_left = False

            for neighbor in neighbors:
                x_neighbor, y_neighbor = self.positions[neighbor]
                if (x_neighbor == x and y < y_neighbor) or 'N' in question_neighbors:  # neighbor is above
                    n_up = True
                    for y_mid in np.arange(y+1, y_neighbor-1):
                        self.ax.plot([x - 0.5, x - 0.5], [y_mid - 0.5, y_mid + 0.5], color='black', linewidth=4)
                        self.ax.plot([x + 0.5, x + 0.5], [y_mid - 0.5, y_mid + 0.5], color='black', linewidth=4)
                if (x_neighbor == x and y > y_neighbor) or 'S' in question_neighbors:  # neighbor is below
                    n_down = True
                    for y_mid in np.arange(y_neighbor + 1, y):
                        self.ax.plot([x - 0.5, x - 0.5], [y_mid - 0.5, y_mid + 0.5], color='black', linewidth=4)
                        self.ax.plot([x + 0.5, x + 0.5], [y_mid - 0.5, y_mid + 0.5], color='black', linewidth=4)
                if (y_neighbor == y and x < x_neighbor) or 'E' in question_neighbors:  # neighbor is right
                    n_right = True
                    for x_mid in np.arange(x+1, x_neighbor-1):
                        self.ax.plot([x_mid - 0.5, x_mid + 0.5], [y - 0.5, y - 0.5], color='black', linewidth=4)
                        self.ax.plot([x_mid - 0.5, x_mid + 0.5], [y + 0.5, y + 0.5], color='black', linewidth=4)
                if (y_neighbor == y and x > x_neighbor) or 'O' in question_neighbors:  # neighbor is left
                    n_left = True
                    for x_mid in np.arange(x_neighbor + 1, x):
                        self.ax.plot([x_mid - 0.5, x_mid + 0.5], [y - 0.5, y - 0.5], color='black', linewidth=4)
                        self.ax.plot([x_mid - 0.5, x_mid + 0.5], [y + 0.5, y + 0.5], color='black', linewidth=4)

            if not n_up:
                self.ax.plot([x - 0.5, x + 0.5], [y + 0.5, y + 0.5], color='black', linewidth=4)
            if not n_down:
                self.ax.plot([x - 0.5, x + 0.5], [y - 0.5, y - 0.5], color='black', linewidth=4)
            if not n_right:
                self.ax.plot([x + 0.5, x + 0.5], [y - 0.5, y + 0.5], color='black', linewidth=4)
            if not n_left:
                self.ax.plot([x - 0.5, x - 0.5], [y - 0.5, y + 0.5], color='black', linewidth=4)

        # Set axis limits
        x = [x for x, _ in self.positions.values()]
        y = [y for _, y in self.positions.values()]
        x_diff = max(x) - min(x)
        y_diff = max(y) - min(y)
        max_diff = max(x_diff, y_diff)

        if max_diff == 0:
            max_diff = 1

        self.ax.set_xlim(min(x) - 0.1 * max_diff, max(x) + 0.1 * max_diff)
        self.ax.set_ylim(min(y) - 0.1 * max_diff, max(y) + 0.1 * max_diff)

        plt.draw()
        plt.pause(0.5)  # Pause to allow update

    def plot_checkerboard(self):
        # Determine the checkerboard size based on node positions
        x_coords = [pos[0] for pos in self.positions.values()]
        y_coords = [pos[1] for pos in self.positions.values()]
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        
        # Create checkerboard pattern
        for x in np.arange(min_x, max_x + 1):
            for y in np.arange(min_y, max_y + 1):
                color = 'white' if (x + y) % 2 == 0 else 'lightgray'
                self.ax.add_patch(plt.Rectangle((x - 0.5, y - 0.5), 1, 1, edgecolor='black', facecolor=color))

    def show(self):
        plt.ioff()  # Turn off interactive mode
        plt.show()

    
    # dijkstra
    def find_shortest_path(self, start):
        import heapq

        distances = {node: float('inf') for node in self.nodes}
        previous_nodes = {node: None for node in self.nodes}
        distances[start] = 0
        priority_queue = [(0, start)]

        while priority_queue:
            current_distance, current_node = heapq.heappop(priority_queue)

            if current_distance > distances[current_node]:
                continue

            for neighbor, weight in self.adjacency_mat[current_node].items():
                distance = current_distance + weight

                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous_nodes[neighbor] = current_node
                    heapq.heappush(priority_queue, (distance, neighbor))

        return distances, previous_nodes

    def get_path(self, from_node, to_node):
        _, previous_nodes = self.find_shortest_path(from_node)
        path = []
        current_node = to_node
        while current_node is not None:
            path.append(current_node)
            current_node = previous_nodes[current_node]
        # path.pop(0)
        return path[::-1]
    
    def print_graph_code_to_build(self):
        print("graph = Graph()")
        for id, node in self.nodes.items():
            x, y = node.get_position()
            print(f"graph.add_node('{id}', {x}, {y})")
            positions = [1 if node.get_neighbor(dir)[0] != 'X' else 0 for dir in ['N', 'S', 'E', 'O']]
            print(f"graph.node_information('{id}', {positions})")
            for dir, (id_neighbor, dist) in node.neighbors.items():
                if id_neighbor != 'X' and id_neighbor != '?':
                    print(f"graph.add_edge('{id}', '{id_neighbor}', '{dir}', {dist})")
    