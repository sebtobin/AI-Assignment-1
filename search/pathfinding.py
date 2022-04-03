import heapq
from search.util import print_coordinate


class Node:
    """
    Class for a graph node, where the node represents a candidate tile for the solution path.

    Duplicate Node objects may exist during A* search and graph traversal utilises constrains of cachex board
    whereby each Node can have upto 6 adjacent nodes each with edge weight 1.
    """
    def __init__(self, r, q):
        self.r = r
        self.q = q

    def __eq__(self, other):
        return self.r == other.r and self.q == other.q

    def __hash__(self):
        return hash((self.r, self.q))

    def get_adjacent_nodes(self):
        """
        Returns a list of 6 node objects, each corresponding to the tiles on the cachex board adjacent to the tile
        associated with the node object. Does not exclude nodes whose coordinates are outside board boundaries.
        """
        adjacent_nodes = []
        # Going diagonally towards bottom left
        adjacent_nodes.append(Node(self.r - 1, self.q))
        # Going diagonally towards top right
        adjacent_nodes.append(Node(self.r + 1, self.q))
        # Going horizontally towards left
        adjacent_nodes.append(Node(self.r, self.q - 1))
        # Going horizontally towards right
        adjacent_nodes.append(Node(self.r, self.q + 1))
        # Going diagonally towards top left
        adjacent_nodes.append(Node(self.r + 1, self.q - 1))
        # Going diagonally towards bot right
        adjacent_nodes.append(Node(self.r - 1, self.q + 1))
        return adjacent_nodes

    def heuristic(self, goal_node):
        """
        The heuristic function used is axial distance. The formula to calculate axial distance of 2 tiles was studied
        and referenced from https://www.redblobgames.com/grids/hexagons/#distances. The axial distance is
        based on the Manhattan distance and is effectively the cost of moving 1 tile on the board to another tile
        (the goal in this case) in a hexagonal grid. The admissibility of this heuristic will be described and verified
        in the report.
        """
        return (abs(self.r - goal_node.r) +
                abs(self.q - goal_node.q) +
                abs(self.r - goal_node.r + self.q - goal_node.q))/2

    def in_bounds(self, n):
        """
        Returns True if the node is within cachex board boundaries.
        """
        return (0 <= self.r < n) and (0 <= self.q < n)

    def is_occupied(self, data):
        """
        Returns True if the node has coordinates equal to any already occupied tiles in board input.
        """
        for node in data["board"]:
            if self.r == node[1] and self.q == node[2]:
                return True
        return False

    def print_node_coordinate(self):
        """
        Prints the coordinate of the node.
        """
        print_coordinate(self.r, self.q)


class NodeCost:
    """
    Stores a Node and both its cumulative path cost, and its heuristic value.
    Contains total cost method used within comparison method for use within
    priority queue and
    """
    def __init__(self, node, cumulative_path_cost, goal_node):
        self.node = node
        self.cumulative_path_cost = cumulative_path_cost
        self.heuristic = self.node.heuristic(goal_node)

    def __lt__(self, other):
        return self.total_cost() < other.total_cost()

    def total_cost(self):
        return self.cumulative_path_cost + self.heuristic


class PriorityQueue:
    """
    Priority queue class which stores objects in a heap and calls __lt__
    method for comparison
    """
    def __init__(self, type):
        self.heap = []
        self.type = type

    """
    Method for inserting object into PQ, must be of certain type.
    """
    def insert_obj(self, obj):
        if isinstance(obj, self.type):
            heapq.heappush(self.heap, obj)
        else:
            #error handling tbd
            pass

    """
    Method for popping minimum object as defined by __lt__ operator.
    """
    def pop_min(self):
        return heapq.heappop(self.heap)

    """
    Returns True if PQ is empty.
    """
    def is_empty(self):
        return len(self.heap) == 0


def search_path(data):
    """
    Takes in data read from a .json file and performs A* search. The A* algorithm implementation was referenced from
    the website https://www.redblobgames.com/pathfinding/a-star/introduction.html#astar with several adaptations
    such as using a heap to implement PriorityQueue as well as Node and NodeCost classes. There also has
    been modifications made using the rules from the Cachex specification and the Project A specification in order
    to make certain nodes invalid such as tiles which have been occupied.
    """

    start_coordinates = data["start"]
    start_node = Node(start_coordinates[0], start_coordinates[1])

    goal_coordinates = data["goal"]
    goal_node = Node(goal_coordinates[0], goal_coordinates[1])

    start_node_cost = NodeCost(start_node, 0, goal_node)
    pq = PriorityQueue(type(start_node_cost))
    pq.insert_obj(start_node_cost)

    came_from_dict = {start_node: None}
    cumulative_path_cost_dict = {start_node: 0}

    while not pq.is_empty():
        # cur_node_cost is an object of NodeCost class which stores an object of Node class, it's path cost and
        # it's heuristic value. cur_node is the aforementioned object of Node class corresponding to cur_node_cost.
        cur_node_cost = pq.pop_min()
        cur_node = cur_node_cost.node

        # We have reached the goal node and have found a solution.
        if cur_node == goal_node:
            break

        # For each node, the average branching factor is 6, as there are 6 possible adjacent nodes to move to.
        for adjacent_node in cur_node.get_adjacent_nodes():

            # If an adjacent node is out of bounds or is already occupied as per the data read in from
            # sample_input.json, then ignore this node and move on to the next one.
            if (not adjacent_node.in_bounds(data["n"]) or adjacent_node.is_occupied(data)):
                continue

            # Path cost of one node to the other is always 1. Find the cumulative_path_cost of the current node
            # add 1 to it and this is the new_cost of this adjacent node.
            new_cost = cumulative_path_cost_dict[cur_node] + 1

            # If this node isn't in cumulative_cost_dict, we have not visited it yet. If the new_cost is less
            # than the value in the cumulative_cost_dict, that means we found a better route to this node.
            # In both cases, we choose to explore that node.
            if adjacent_node not in cumulative_path_cost_dict or new_cost < cumulative_path_cost_dict[adjacent_node]:
                cumulative_path_cost_dict[adjacent_node] = new_cost

                # When a node is inserted into the priority queue, a NodeCost object corresponding to it will be
                # created and heap.heapify will use the __lt__ comparator method of the object to do heapsort.
                adj_node_cost = NodeCost(adjacent_node, new_cost, goal_node)
                pq.insert_obj(adj_node_cost)
                came_from_dict[adjacent_node] = cur_node

    path = []
    # If cur_node is determined to be goal_node outside the A* while loop, that means that break was
    # called in the while loop and that a solution is found. If goal node was not reached, the path is empty and
    # the length of the path is 0.w
    if cur_node == goal_node:
        # cur_node is the goal node at the start of the while loop, and from the goal node, go backwards and get
        # each node expanded to reach the goal node. In a tree representation, this is essentially going back up
        # the tree from the goal node.
        # Insert each of these nodes into index 0 in the path list such that a sequential path of nodes from the
        # start_node to the goal_node is formed through index 0 to d in the list.
        while cur_node is not None:
            path.insert(0, cur_node)
            cur_node = came_from_dict[cur_node]


    print(len(path))
    for node in path:
        node.print_node_coordinate()

