import math
import time
from functools import wraps
import random
import rclpy
from simple_arm_demo.collision_checker import CollisionChecker
from simple_arm_demo.node import Node
from nav_msgs.msg import Path
import heapq
from rclpy.time import Time
from time import sleep

def timeit(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        total_time = end_time - start_time
        rclpy.logging.get_logger("PRM motion planner").info(
            f"Function {func.__name__}{args} {kwargs} Took {total_time:.4f} seconds"
        )
        return result

    return timeit_wrapper


class LazyPRM:
    """PRM algorithm implementation as a planner to solve start-to-goal queries"""

    def __init__(self):
        # random.seed(42)

        self.start: Node = None
        self.goal: Node = None

        self.step_size = 0.001
        self.neighbor_distance = 0.5

        self.collision_checker: CollisionChecker = None
        self.tolerance = 0.001

        self.graph = {}
        self.connections = set()

        self.solution_node = None

        self.solution_path = None

        # astar attributes
        self.open_list = []
        self.closed_list = []
        self.g = {}
        self.nodes = {}

    def set_start(self, start):
        """Set start position of the robot"""
        self.start = start

    def set_goal(self, goal):
        """Set goal of the start to goal query"""
        self.goal = goal

    def set_collision_checker(self, collision_checker):
        """Set collision checker to detect collision free nodes"""
        self.collision_checker = collision_checker

    def set_step_size(self, step_size):
        """Set the step size to determine if connection between two
        nodes is possible with collision checking"""
        self.step_size = step_size

    def set_neighbor_distance(self, neighbor_distance):
        """Set the maximum expansion for a new node in the tree"""
        self.neighbor_distance = neighbor_distance

    def get_solution_path(self):
        """Obtain stored solution path"""
        return self.solution_path

    def get_path_length(self):
        """Obtain the length of the solution path"""
        distance = 0

        for i in range(0, len(self.solution_path) - 2):
            distance += math.sqrt(
                math.pow(
                    self.solution_path[i].theta1 - self.solution_path[i + 1].theta1, 2
                )
                + math.pow(
                    self.solution_path[i].theta2 - self.solution_path[i + 1].theta2, 2
                )
            )

        return distance

    def get_graph(self):
        return self.connections

    @timeit
    def solve(self) -> list:
        """Function to find path from start to goal using Dijkstra algorithm"""

        if not self.collision_checker:
            rclpy.logging.get_logger("PRM motion planner").warn(
                "Collision checker has not been set on the planner"
            )
            return False
        if not self.start:
            rclpy.logging.get_logger("PRM motion planner").warn(
                "Start position has not been set on the planner"
            )
            return False
        if not self.goal:
            rclpy.logging.get_logger("PRM motion planner").warn(
                "Goal position has not been set on the planner"
            )
            return False

        self.open_list = []
        self.closed_list = []
        self.g = {}
        self.nodes = {}

        # finding nearest node from start
        start_coord = None
        init_dis = float("inf")

        for node in self.graph:
            cur_dis = self.graph[node].calculate_distance(self.start)
            if cur_dis < init_dis:
                if self.collision_checker.is_connection_free(
                    self.graph[node], self.start, self.step_size
                ):
                    start_coord = node
                    init_dis = cur_dis
        if start_coord is None:
            rclpy.logging.get_logger("PRM motion planner").warn(
                "Could not connect the start to a roadmap."
            )
            return False

        # finding nearest node from goal
        goal_coord = None
        init_dis = float("inf")

        for node in self.graph:
            cur_dis = self.graph[node].calculate_distance(self.goal)
            if cur_dis < init_dis:
                if self.collision_checker.is_connection_free(
                    self.graph[node], self.goal, self.step_size
                ):
                    goal_coord = node
                    init_dis = cur_dis
        if goal_coord is None:
            rclpy.logging.get_logger("PRM motion planner").warn(
                "Could not connect the goal to a roadmap."
            )
            return False

        # #########################

        # variables for collision checking
        nodes_in_collision = []
        invalid_connections = []
        node_to_remove = False
        connections_to_remove = False
        collision_free = True

        start_node = self.graph[start_coord]

        goal_node = self.graph[goal_coord]

        self.g[(start_node.theta1, start_node.theta2)] = 0
        self.g[(goal_node.theta1, goal_node.theta2)] = math.inf

        heapq.heappush(self.open_list, (0.0, (start_node.theta1, start_node.theta2))) # Assign open_list the starting node's x,y coords with cost value 0.0
        self.nodes[(start_node.theta1, start_node.theta2)] = start_node

        while len(self.open_list) > 0:                      # While there are nodes left to check:
            temp_node = heapq.heappop(self.open_list)[1]    # make temp_node the (x,y) tuple of current node as storage. [0] would be cost to get there, [1] is coords
            current_node = self.nodes[temp_node]            # current node is node object retreived via (x,y) tuple key.

            for connection in self.graph[                   # get current node object from the graph and find all it's connections
                (current_node.theta1, current_node.theta2)  # These are just x and y coords. (theta1 = x) (theta2 = y)   
            ].connections.items():
                # If the distance from the connection to the goal_node is within the tolerance, it means the path is complete.
                if (connection[1].calculate_distance(goal_node) < self.tolerance):  # Found path

                    connection[1].parent = current_node
                    temp_path = connection[1].backtrack_path()

                    if len(temp_path) > 0: 
                    # Collision check the found path
                        
                        # First check nodes used in the path are actually valid positions on the map
                        nodes_in_collision = self.collision_checker.is_path_free(temp_path)    # nodes_in_collision is a list of [(x,y)] tuples
                        if len(nodes_in_collision) > 0:     # If this list contains any entries, there are still unusable nodes to be removed
                            collision_free = False
                            node_to_remove = True           # Add flag to actually remove these nodes later
                            rclpy.logging.get_logger("PRM motion planner").warn(
                            "There are nodes in collision on found path")
                            break

                        # Once solution_path has no invalid nodes, check connections between selected nodes.
                        else:
                            should_break = False
                            for current_n1_coord, next_n2_coord in zip(temp_path, temp_path[1:]):
                            # ^^ current_n1_coord and current_n2_coord are node objects from the found solution path ^^
                                if not (self.collision_checker.is_connection_free(current_n1_coord, next_n2_coord, self.step_size )): # If the connection collides with something
                                    collision_free = False
                                    connections_to_remove = True
                                    should_break = True     # Breaks out of the connections in node loop, stays inside while loop checking all nodes
                                    nodes_in_tuple = (current_n1_coord, next_n2_coord)
                                    invalid_connections.append(nodes_in_tuple)      # Add nodes with bad connection to list to remove connection later
                            rclpy.logging.get_logger("PRM motion planner").warn(
                            "There are connections that need to be removed")
                            if should_break:
                                break

                # At this point. Solution path will be collision free (=True) or need to break out of loop, remove nodes and run again with updated graph.

                        if collision_free:
                            # Originial code, but only attempted solution paths have been checked for collisions. Current path has not tripped any collision_free flags.
                            self.solution_path = ([self.start] + temp_path + [self.goal])
                            return True

                    else:
                        return False  # Because solution path has no length so is wrong?

# Rest of algorithm -----------------------------------------------

                new_cost = current_node.g + connection[1].calculate_distance(
                    current_node
                )

                if (connection[1].theta1, connection[1].theta2) not in self.g:
                    self.g[(connection[1].theta1, connection[1].theta2)] = math.inf     # If connected node cost from current node is not recorded yet, set it to infinity

                if new_cost < self.g[(connection[1].theta1, connection[1].theta2)]:     # If the new cost to get to the connected node is less than last known cost to get to same node, replace it
                    self.g[(connection[1].theta1, connection[1].theta2)] = new_cost
                    connection[1].g = new_cost
                    connection[1].h = connection[1].calculate_distance(self.goal)
                    connection[1].f = connection[1].g + connection[1].h
                    connection[1].parent = current_node

                    heapq.heappush(
                        self.open_list,
                        (
                            connection[1].f,
                            (connection[1].theta1, connection[1].theta2),
                        ),
                    )
                    self.nodes[
                        (connection[1].theta1, connection[1].theta2)
                    ] = connection[1]

            
            ####### Node removal here

            if not collision_free:
            # Remove the nodes or connections as needed, then run the function again.

                if node_to_remove:
                    for node in nodes_in_collision:     # node here is an (x,y) tuple 
                        if node in self.graph:          # Check node is in graph first, should be redundant check but happens just in case and to get error message

                            deleted_node_connections = self.graph[node].connections     # deleted_node_connections is a dictionary
                            
                            # Iterate through all the connections' connected dictionarys removing the node to be deleted from them
                            for connected_node in deleted_node_connections:         # connected_node is tuple dictionary key
                                if ( (node, connected_node) in self.connections ):                              
                                    self.connections.remove((node, connected_node)) # These two 'if's are crash protection for when multiple nodes in the
                                if ( (connected_node, node) in self.connections ):  # solution path are invalid as the connection would already be gone
                                    self.connections.remove((connected_node, node))
                                
                                del self.graph[connected_node].connections[node] # Remove the invalid node from other node's dictionary

                            del self.graph[node] # Delete node itself from graph.

                elif connections_to_remove:
                    for nodes_in_a_tuple in invalid_connections: # nodes_in_a_tuple is in fact two node objects... you will never guess... inside a tuple!!
                        node1 = nodes_in_a_tuple[0]
                        node2 = nodes_in_a_tuple[1]
                        n1_coord = (node1.theta1, node1.theta2)
                        n2_coord = (node2.theta1, node2.theta2)

                        if ( (n1_coord, n2_coord) in self.connections ):
                            self.connections.remove((n1_coord, n2_coord))   # 'if' checks are safety nets

                        if ( (n2_coord, n1_coord) in self.connections ):
                            self.connections.remove((n2_coord, n1_coord))

                        del node1.connections[(node2.theta1, node2.theta2)]
                        del node2.connections[(node1.theta1, node1.theta2)]

                return self.solve()

            self.closed_list.append(current_node)

        return False

        # ################################

    @timeit
    def generate_samples(self, samples_number):
        for i in range(samples_number):
            nrand = self.sample()
            # if self.collision_checker.is_node_free(nrand):
            if (nrand.theta1, nrand.theta2) not in self.graph:
                self.graph[(nrand.theta1, nrand.theta2)] = nrand

    @timeit
    def generate_connections(self):
        for n1_coord in self.graph:
            _n1 = self.graph[n1_coord]
            for n2_coord in self.graph:
                if (
                    n1_coord != n2_coord
                    and n2_coord not in self.graph[n1_coord].connections
                ):
                    if _n1.calculate_distance(
                        self.graph[n2_coord]
                    ) <= self.neighbor_distance:
                        self.graph[n1_coord].connections[n2_coord] = self.graph[
                            n2_coord
                        ]
                        self.graph[n2_coord].connections[n1_coord] = self.graph[
                            n1_coord
                        ]
                        if (n1_coord, n2_coord) not in self.connections and (
                            n2_coord,
                            n1_coord,
                        ) not in self.connections:
                            self.connections.add((n1_coord, n2_coord))
                            # print("hey here is the coord of random type: ", (n1_coord, n2_coord))

    def sample(self):
        """Sample node from the workspace"""
        _i = int(random.uniform(0, self.collision_checker.map_width - 1))
        _j = int(random.uniform(0, self.collision_checker.map_height - 1))
        _x, _y = self.collision_checker.indices_to_coordinates(_i, _j)
        node = Node(_x, _y)
        return node

    def nodes_to_path_msg(self, path_nodes: list) -> Path:
        """Transforms the list of path nodes into a Path type of object"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = Time().to_msg()

        for node in path_nodes:
            path.poses.append(node.to_pose_stamped())

        return path

    def interpolate_solution(self, step):
        interpolated_path = []

        interpolated_path.append(self.solution_path[0])

        for i in range(len(self.solution_path) - 1):
            interpolated_path.append(self.solution_path[i])
            distance = self.solution_path[i].calculate_distance(
                self.solution_path[i + 1]
            )
            _u = int(distance / step)

            theta = math.atan2(
                self.solution_path[i + 1].theta2 - self.solution_path[i].theta2,
                self.solution_path[i + 1].theta1 - self.solution_path[i].theta1,
            )

            for j in range(1, _u):
                interpolated_path.append(
                    Node(
                        self.solution_path[i].theta1 + step * j * math.cos(theta),
                        self.solution_path[i].theta2 + step * j * math.sin(theta),
                    )
                )

        self.solution_path = interpolated_path
