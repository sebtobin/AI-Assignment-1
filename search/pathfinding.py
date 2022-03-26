import heapq


class Node:
    def __init__(self, coordinate_tuple):
        self.coordinate_q = coordinate_tuple[0]
        self.coordinate_r = coordinate_tuple[1]

    def heuristic(self, goal_node):
        return (abs(self.coordinate_q - goal_node.coordinate_q) +
                abs(self.coordinate_r - goal_node.coordinate_r) +
                abs(self.coordinate_q - goal_node.coordinate_q + self.coordinate_r - goal_node.coordinate_r) / 2)

    def get_coordinate_tuple(self):
        coordinate_tuple = (self.coordinate_q, self.coordinate_r)
        return coordinate_tuple


class CoordinateHeuristicPair:

    def __init__(self, node_to_insert, goal_node):
        self.node = node_to_insert
        self.heuristic_val = self.node.heuristic(goal_node)

    def __lt__(self, other):
        return self.heuristic_val < other.heuristic_val


class PriorityQueue:
    def __init__(self):
        self.heap = []

    def insert_node(self, node_to_insert, goal_node):
        cur_node_coordinate_heuristic_pair = CoordinateHeuristicPair(node_to_insert, goal_node)
        self.heap.append(cur_node_coordinate_heuristic_pair)
        heapq.heapify(self.heap)

    def pop_min(self):
        return heapq.heappop(self.heap)

    def is_empty(self):
        return len(self.heap) == 0


# Takes in the coordinates of the start and goal.
def search_path(start_coordinates, goal_coordinates):
    pq = PriorityQueue()
    start_node = Node(start_coordinates)
    goal_node = Node(goal_coordinates)
    pq.insert_node(start_node, goal_node)
