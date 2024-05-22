import random
import networkx as nx
import matplotlib.pyplot as plt
import heapq

def swap_dir_array(arr, dir):
    N, S, E, O = arr
    match dir:
        case "N":
            return [N, S, O, E]
        case "S":
            return [S, N, E, O]
        case "E":
            return [E, O, N, S]
        case "O":
            return [O, E, S, N]

class Node:
    def __init__(self, id):
        self.id = id
        self.position = (0, 0)
        self.neighbors = {'N': ['X', 0], 'S': ['X', 0], 'E': ['X', 0], 'O': ['X', 0]}

    def set_neighbor(self, dir, id_neighbor):
        if dir in self.neighbors:
            self.neighbors[dir] = id_neighbor

    def get_neighbor(self, dir):
        return self.neighbors[dir]

    def set_position(self, x, y):
        self.position = (x, y)

    def get_position(self):
        return self.position

    def is_complete(self):
        return all([id != '?' for id, _ in self.neighbors.values()])

class Graph:
    def __init__(self, grid_size=20):
        self.nodes = {}
        self.adjacency_mat = {}
        self.unexplored = []
        self.G = nx.Graph()
        self.positions = {}
        self.not_info = []
        self.walls = set()
        self.grid_size = grid_size
        self.fig, self.ax = plt.subplots()

    def add_node(self, id, x, y):
        node = Node(id)
        node.set_position(x, y)
        self.nodes[id] = node
        self.adjacency_mat[id] = {}
        self.positions[id] = (x, y)
        self.G.add_node(id, pos=(x, y))

    def node_information(self, id, positions):
        for i, pos in enumerate(positions):
            if pos == 1:
                dir = ['N', 'S', 'E', 'O'][i]
                if self.nodes[id].get_neighbor(dir)[0] == 'X':
                    self.add_unexplored(id, dir)
                    self.nodes[id].set_neighbor(dir, ('?', 0))

        if id in self.not_info:
            self.not_info.remove(id)
        
        self.update_plot()

    def add_unexplored(self, id, dir):
        self.nodes[id].set_neighbor(dir, ('?', 0))
        self.unexplored.append((id, dir))

    def exist_closed_node(self, prev_node, direction, distance, neighbor, threshold=0.1):
        pos_dist = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'O': (-1, 0)}[direction]
        x, y = self.nodes[prev_node].get_position()
        pos_dist = (pos_dist[0] * distance, pos_dist[1] * distance)
        x += pos_dist[0]
        y += pos_dist[1]
        for id, node in self.nodes.items():
            x_node, y_node = node.get_position()
            if abs(x_node - x) < threshold and abs(y_node - y) < threshold:
                neighbor_node = [1 if node.get_neighbor(dir)[0] != 'X' else 0 for dir in ['N', 'S', 'E', 'O']]
                neighbor = swap_dir_array(neighbor, direction)
                if neighbor_node == neighbor:
                    return id
        return None

    def add_edge(self, id1, id2, direction, distance):
        if id2 not in self.nodes:
            pos_dist = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'O': (-1, 0)}[direction]
            x, y = self.nodes[id1].get_position()
            pos_dist = (pos_dist[0] * distance, pos_dist[1] * distance)
            x += pos_dist[0]
            y += pos_dist[1]
            self.add_node(id2, x, y)
            self.not_info.append(id2)

        self.adjacency_mat[id1][id2] = distance
        self.adjacency_mat[id2][id1] = distance

        opposite_direction = {'N': 'S', 'S': 'N', 'E': 'O', 'O': 'E'}[direction]
        self.nodes[id1].set_neighbor(direction, (id2, distance))
        self.nodes[id2].set_neighbor(opposite_direction, (id1, distance))

        if (id2, opposite_direction) in self.unexplored:
            self.unexplored.remove((id2, opposite_direction))

        if (id1, direction) in self.unexplored:
            self.unexplored.remove((id1, direction))

        self.G.add_edge(id1, id2)
        self.update_plot()

    def get_direction_unexplored(self, id, direction):
        opposite = {'N': 'S', 'S': 'N', 'E': 'O', 'O': 'E'}[direction]
        for dir, (id_neighbor, _) in self.nodes[id].neighbors.items():
            if id_neighbor == '?' and dir != opposite:
                return dir
        return None

    def closest_unexplored(self, id):
        if not self.nodes[id].is_complete():
            for dir, (id_neighbor, _) in self.nodes[id].neighbors.items():
                if id_neighbor == '?':
                    return id, dir
                
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
        self.ax.clear()
        node_color = ['blue' if id not in self.not_info else 'black' for id in self.nodes]
        node_color = ['red' if id in [n for n, _ in self.unexplored] else color for id, color in zip(self.nodes, node_color)]
        nx.draw(self.G, self.positions, with_labels=True, node_size=1000, node_color=node_color, font_size=10, font_weight='bold', font_color='black', edge_color='black', width=2, ax=self.ax)

        for id, dir in self.unexplored:
            x, y = self.positions[id]
            dx, dy = {'N': (0, 0.5), 'S': (0, -0.5), 'E': (0.5, 0), 'O': (-0.5, 0)}[dir]
            self.ax.arrow(x, y, dx, dy, head_width=0.1, head_length=0.2, fc='gray', ec='gray')

        x = [x for x, _ in self.positions.values()]
        y = [y for _, y in self.positions.values()]
        x_diff = max(x) - min(x)
        y_diff = max(y) - min(y)
        max_diff = max(x_diff, y_diff)
        if max_diff == 0:
            max_diff = 1
        self.ax.set_xlim(min(x) - 0.5 * max_diff, max(x) + 0.5 * max_diff)
        self.ax.set_ylim(min(y) - 0.5 * max_diff, max(y) + 0.5 * max_diff)

        self.plot_walls_and_board()
        plt.draw()
        plt.pause(0.1)

    def plot_walls_and_board(self):
        for i in range(self.grid_size):
            for j in range(self.grid_size):
                self.ax.add_patch(plt.Rectangle((i - 0.5, j - 0.5), 1, 1, edgecolor='black', facecolor='none'))

        for (x1, y1), (x2, y2) in self.walls:
            self.ax.plot([x1, x2], [y1, y2], color='red', linewidth=2)

    def show(self):
        plt.ioff()
        plt.show()

    def find_shortest_path(self, start):
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
        while current_node:
            path.append(current_node)
            current_node = previous_nodes[current_node]
        return path[::-1]

    def generate_maze(self):
        self.add_node('A', 0, 0)
        stack = [('A', (0, 0))]
        visited = set()
        visited.add('A')

        directions = ['N', 'S', 'E', 'O']
        opposite = {'N': 'S', 'S': 'N', 'E': 'O', 'O': 'E'}
        moves = {'N': (0, 1), 'S': (0, -1), 'E': (1, 0), 'O': (-1, 0)}

        while stack:
            current, (x, y) = stack[-1]
            neighbors = [(current, d, moves[d], opposite[d]) for d in directions]
            random.shuffle(neighbors)

            moved = False
            for current, direction, move, opp in neighbors:
                nx, ny = x + move[0], y + move[1]
                if 0 <= nx < self.grid_size and 0 <= ny < self.grid_size:
                    neighbor_id = f"{nx},{ny}"
                    if neighbor_id not in visited:
                        self.add_node(neighbor_id, nx, ny)
                        self.add_edge(current, neighbor_id, direction, 1)
                        self.add_wall(x, y, direction)
                        visited.add(neighbor_id)
                        stack.append((neighbor_id, (nx, ny)))
                        moved = True
                        break

            if not moved:
                stack.pop()

        self.update_plot()

    def add_wall(self, x, y, direction):
        if direction == 'N':
            self.walls.add(((x - 0.5, y + 0.5), (x + 0.5, y + 0.5)))
        elif direction == 'S':
            self.walls.add(((x - 0.5, y - 0.5), (x + 0.5, y - 0.5)))
        elif direction == 'E':
            self.walls.add(((x + 0.5, y - 0.5), (x + 0.5, y + 0.5)))
        elif direction == 'O':
            self.walls.add(((x - 0.5, y - 0.5), (x - 0.5, y + 0.5)))

graph = Graph(grid_size=20)
plt.show(block=False)

graph.generate_maze()

start_node = '0,0'
end_node = f"{graph.grid_size - 1},{graph.grid_size - 1}"

print(f"Shortest path from {start_node} to {end_node}:")
print(graph.get_path(start_node, end_node))

graph.show()
