import heapq


class Node:
    def __init__(self, q, r):
        self.q = q
        self.r = r

    def __eq__(self, other):
        return self.q == other.q and self.r == other.r

    def __hash__(self):
        return hash((self.q, self.r))

    def get_adjacent_nodes(self):
        adjacent_nodes = []
        adjacent_nodes.append(Node(self.q, self.r - 1))
        adjacent_nodes.append(Node(self.q, self.r + 1))
        adjacent_nodes.append(Node(self.q - 1, self.r))
        adjacent_nodes.append(Node(self.q - 1, self.r + 1))
        adjacent_nodes.append(Node(self.q + 1, self.r - 1))
        adjacent_nodes.append(Node(self.q + 1, self.r))
        return adjacent_nodes

    def heuristic(self, goal_node):
        return (abs(self.q - goal_node.q) +
                abs(self.r - goal_node.r) +
                abs(self.q - goal_node.q + self.r - goal_node.r))/2

    def get_coordinate_tuple(self):
        coordinate_tuple = (self.q, self.r)
        return coordinate_tuple


# Stores a Node and both its cumulative path cost, and its heuristic
class NodeCost:
    def __init__(self, node, cumulative_path_cost, goal_node):
        self.node = node
        self.cumulative_path_cost = cumulative_path_cost
        self.heuristic = self.node.heuristic(goal_node)

    def __lt__(self, other):
        return self.total_cost() < other.total_cost()

    def total_cost(self):
        return self.cumulative_path_cost + self.heuristic


class PriorityQueue:
    def __init__(self):
        self.heap = []

    def insert_node(self, node_to_insert, cumulative_path_cost, goal_node):
        cur_node_cost = NodeCost(node_to_insert, cumulative_path_cost, goal_node)
        self.heap.append(cur_node_cost)
        heapq.heapify(self.heap)

    def pop_min(self):
        return heapq.heappop(self.heap)

    def is_empty(self):
        return len(self.heap) == 0


# Takes in the coordinates of the start and goal.
def search_path(data):
    pq = PriorityQueue()

    start_coordinates = data["start"]
    start_node = Node(start_coordinates[0], start_coordinates[1])

    goal_coordinates = data["goal"]
    goal_node = Node(goal_coordinates[0], goal_coordinates[1])

    pq.insert_node(start_node, 0, goal_node)

    came_from_dict = {start_node: None}
    cumulative_cost_dict = {start_node: 0}

    cur_node = None

    while not pq.is_empty():
        # cur_node_cost is an object of NodeCost class which stores an object of Node class, it's path cost and
        # it's heuristic value. cur_node is the aforementioned object of Node class corresponding to cur_node_cost.
        cur_node_cost = pq.pop_min()
        cur_node = cur_node_cost.node

        # We have reached the goal node and have found a solution.
        if cur_node == goal_node:
            break

        for adjacent_node in cur_node.get_adjacent_nodes():

            # If an adjacent node is out of bounds or is already occupied as per the data read in from
            # sample_input.json, then ignore this node and move on to the next one.
            if ((not in_bounds(data["n"], adjacent_node.q, adjacent_node.r) or
                is_occupied(data, adjacent_node.q, adjacent_node.r))):
                continue

            # Path cost of one node to the other is always 1. Find the cumulative_path_cost of the current node
            # add 1 to it and this is the new_cost of this adjacent node.
            new_cost = cumulative_cost_dict[cur_node] + 1

            # If this node isn't in cumulative_cost_dict, we have not visited it yet. If the new_cost is less
            # than the value in the cumulative_cost_dict, that means we found a better route to this node.
            # In both cases, we choose to explore that node.
            if adjacent_node not in cumulative_cost_dict or new_cost < cumulative_cost_dict[adjacent_node]:
                cumulative_cost_dict[adjacent_node] = new_cost

                # When a node is inserted into the priority queue, a NodeCost object corresponding to it will be
                # created and heap.heapify will use the __lt__ comparator method of the object to do heapsort.
                pq.insert_node(adjacent_node, new_cost, goal_node)
                came_from_dict[adjacent_node] = cur_node

    # a solution was found
    if cur_node == goal_node:
        path = []
        while cur_node is not None:
            path.insert(0, cur_node.get_coordinate_tuple())
            cur_node = came_from_dict[cur_node]
        print(str(len(path)))
        for coord in path:
            print(str(coord))
    # no solution was found
    else:
        # tbd
        pass


def in_bounds(n, q, r):
    return (0 <= q < n) and (0 <= r < n)


def is_occupied(data, q, r):
    if q == 1 and r == 0:
        print()
    for node in data["board"]:
        if q == node[1] and r == node[2]:
            return True
    return False
