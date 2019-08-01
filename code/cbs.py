import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, paths_violate_constraint, get_location, get_sum_of_cost


def detect_conflict(path1, path2):
    ##############################
    # Task 3.1: Return the first conflict that occurs between two robot paths (or None if there is no conflict)
    #           There are two types of conflicts: vertex conflict and edge conflict.
    #           A vertex conflict occurs if both robots occupy the same location at the same timestep
    #           An edge conflict occurs if the robots swap their location at the same timestep.
    #           You should use "get_location(path, t)" to get the location of a robot at time t.

    for t in range(max(len(path1), len(path2))):
        c1 = get_location(path1, t)
        c2 = get_location(path2, t)
        if c1 == c2:
            return [c1, t]
        else:
            p1 = get_location(path1, t-1)
            p2 = get_location(path2, t-1)
            if c1 == p2 and c2 == p1:
                return [p1, c1, t]
    return None




def detect_conflicts(paths):
    ##############################
    # Task 3.1: Return a list of first conflicts between all robot pairs.
    #           A conflict can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the conflict, and the timestep at which the conflict occurred.
    #           You should use your detect_conflict function to find a conflict between two robots.
    
    conflicts = []
    for i in range(len(paths) - 1):
        for j in range(i + 1, len(paths)):
            conflict = detect_conflict(paths[i], paths[j])
            if conflict is not None:
                conflicts.append({'a1' : i,
                                  'a2' : j,
                                  'loc' : conflict[: -1],
                                  'timestep': conflict[-1]})
    return conflicts
            


def standard_splitting(conflict):
    ##############################
    # Task 3.2: Return a list of (two) constraints to resolve the given conflict
    #           Vertex conflict: the first constraint prevents the first agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the second agent to be at the
    #                            specified location at the specified timestep.
    #           Edge conflict: the first constraint prevents the first agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the second agent to traverse the
    #                          specified edge at the specified timestep

    constrain1 = {'agent': conflict['a1'],
                  'loc': conflict['loc'],
                  'timestep': conflict['timestep'],
                  'positive': False}
    loc = list(conflict['loc'])
    loc.reverse()
    constrain2 = {'agent': conflict['a2'],
                  'loc': conflict['loc'],
                  'timestep': conflict['timestep'],
                  'positive': False}
    return [constrain1, constrain2]


def disjoint_splitting(conflict):
    ##############################
    # Task 4.1: Return a list of (two) constraints to resolve the given conflict
    #           Vertex conflict: the first constraint enforces one agent to be at the specified location at the
    #                            specified timestep, and the second constraint prevents the same agent to be at the
    #                            same location at the timestep.
    #           Edge conflict: the first constraint enforces one agent to traverse the specified edge at the
    #                          specified timestep, and the second constraint prevents the same agent to traverse the
    #                          specified edge at the specified timestep
    #           Choose the agent randomly

    i = random.randint(0,1)
    if i == 0:
         constrain1 = {'agent': conflict['a1'],
                  'loc': conflict['loc'],
                  'timestep': conflict['timestep'],
                  'positive': True}
         constrain2 = {'agent': conflict['a1'],
                  'loc': conflict['loc'],
                  'timestep': conflict['timestep'],
                  'positive': False}
    else:
        loc = list(conflict['loc'])
        loc.reverse()

        constrain1 = {'agent': conflict['a2'],
                  'loc': loc,
                  'timestep': conflict['timestep'],
                  'positive': True}
        constrain2 = {'agent': conflict['a2'],
                  'loc': loc,
                  'timestep': conflict['timestep'],
                  'positive': False}
    
    return [constrain1, constrain2]


class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['conflicts']), self.num_of_generated, node))
        print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True, time_limit=60):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        time_limit  - maximum amount of execution time allowed
        """

        start_time = timer.time()

        # Generate the root node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # conflicts     - list of conflicts in paths
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'conflicts': [],
                'parent': None}
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'], root['paths'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        root['cost'] = get_sum_of_cost(root['paths'])
        root['conflicts'] = detect_conflicts(root['paths'])
        self.push_node(root)

        # Task 3.1: Testing
        print(root['conflicts'])

        # Task 3.2: Testing
        for conflict in root['conflicts']:
            print(standard_splitting(conflict))

        ##############################
        # Task 3.3: High-Level Search
        #           Repeat the following as long as the open list is not empty:
        #             1. Get the next node from the open list (you can use self.pop_node()
        #             2. If this node has no conflict, return solution
        #             3. Otherwise, choose the first conflict and convert to a list of constraints (using your
        #                standard_splitting function). Add a new child node to your open list for each constraint
        #           Ensure to create a copy of any objects that your child nodes might inherit

        while len(self.open_list) > 0:
            node = self.pop_node()
            if node['conflicts'] == []:
                self.print_results(node)
                return node['paths']
            
            conflict = node['conflicts'][0]
            print("Choose a conflict between {} and {} at location {} at timestep {}".format(
                conflict['a1'], conflict['a2'], conflict['loc'], conflict['timestep']))

            #new_constraints = standard_splitting(conflict)
            new_constraints = disjoint_splitting(conflict)

            for constraint in new_constraints:
                #i = constraint['agent']
                print("Negative constraint on agent {} at location {} at timestep {}".format(
                    constraint['agent'], constraint['loc'], constraint['timestep']))
                constraints = list(node['constraints'])
                constraints.append(constraint)

                paths = list(node['paths'])
                #paths[i] = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                #                    i, constraints, paths)
                
                replan = []
                if constraint['positive']:
                    replan = paths_violate_constraint(constraint, paths)
                else:
                    replan.append(constraint['agent'])
                #child_node = {'cost': get_sum_of_cost(paths),
                #              'constraints':constraints,
                #              'paths': paths,
                #              'conflicts': detect_conflicts(paths),
                #              'parent': node}
                #self.push_node(child_node)

                prune = False
                for i in replan:
                    paths[i] = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                                   i, constraints, paths)
                    
                    if paths[i] is None:
                        prune = True
                        break

                if not prune:
                    child_node = {'cost': get_sum_of_cost(paths), 
                                  'constraints': constraints,
                                  'paths': paths,
                                  'conflicts': detect_conflicts(paths),
                                  'parent': node}
                    self.push_node(child_node)

    def print_results(self, node):
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
