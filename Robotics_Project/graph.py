import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation

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

    def get_neighbor(self, dir):
        return self.neighbors[dir]

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
                print(neighboor_node)
                # correct neighboor using the direction
                neighboor = swap_dir_array(neighboor, direction)
                print("")
                print(neighboor_node)
                print(neighboor)    
                print(neighboor_node == neighboor)
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

        # Add or update edges only
        self.G.add_edge(id1, id2)
        self.update_plot()


    def get_direction_uneplored(self, id, direction):
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

    def update_plot(self):
        self.ax.clear()  # Clears the current axes
        node_color = ['blue' if id not in self.not_info else 'black' for id in self.nodes]
        node_color = ['red' if id in [n for n, _ in self.unexplored] else color for id, color in zip(self.nodes, node_color)]
        nx.draw(self.G, self.positions, with_labels=True, node_size=1000, node_color=node_color, font_size=10, font_weight='bold', font_color='black', edge_color='black', width=2, ax=self.ax)

        for id, dir in self.unexplored:
            x, y = self.positions[id]
            dx, dy = {'N': (0, 0.1), 'S': (0, -0.1), 'E': (0.1, 0), 'O': (-0.1, 0)}[dir]
            self.ax.arrow(x, y, dx, dy, head_width=0.05, head_length=0.1, fc='gray', ec='gray')


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

    def show(self):
        plt.ioff()  # Turn off interactive mode
        plt.show()