from instance_generator import move
import heapq


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            next_loc = move(loc, dir)
            next_cost = cost + 1
            if my_map[next_loc[0]][next_loc[1]]:
                continue
            next = {'loc': next_loc, 'cost': next_cost}
            if next_loc in closed_list:
                existing_node = closed_list[next_loc]
                if existing_node['cost'] > next_cost:
                    closed_list[next_loc] = next
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (next_cost, next_loc, next))
            else:
                closed_list[next_loc] = next
                heapq.heappush(open_list, (next_cost, next_loc, next))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    positive = []  # to collect positive constraints
    negative = []  # to collect negative constraints
    max_timestep = -1  # the maximum timestep in these constraints
    #  collect constraints that are related to this agent
    for constraint in constraints:
        if constraint['positive']:  # positive constraint is effective for everyone
            if constraint['agent'] == agent:
                positive.append(constraint)
            else:
                negative.append(constraint)
            max_timestep = max(max_timestep, constraint['timestep'])
        elif constraint['agent'] == agent:  # negative constraint is effective for only one agent
            negative.append(constraint)
            max_timestep = max(max_timestep, constraint['timestep'])

    constraint_table = [[] for _ in range(max_timestep + 1)]
    for constraint in positive:
        if len(constraint['loc']) == 1:  # positive vertex constraint
            constraint_table[constraint['timestep']].append({'loc': constraint['loc'], 'positive': True})
        else:  # positive edge constraint
            constraint_table[constraint['timestep'] - 1].append({'loc': [constraint['loc'][0]], 'positive': True})
            constraint_table[constraint['timestep']].append({'loc': [constraint['loc'][1]], 'positive': True})

    for constraint in negative:
        if len(constraint['loc']) == 1:  # vertex constraint
            constraint_table[constraint['timestep']].append({'loc': constraint['loc'], 'positive': False})
        elif constraint['positive']:  # positive edge constraint for other agents
            constraint_table[constraint['timestep'] - 1].append({'loc': [constraint['loc'][0]], 'positive': False})
            constraint_table[constraint['timestep']].append({'loc': [constraint['loc'][1]], 'positive': False})
            constraint_table[constraint['timestep']].append(
                {'loc': [constraint['loc'][1], constraint['loc'][0]], 'positive': False})
        else:  # negative edge constraint
            constraint_table[constraint['timestep']].append({'loc': constraint['loc'], 'positive': False})

    return constraint_table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def paths_violate_constraint(constraint, paths):
    assert constraint['positive'] is True
    rst = []
    for i in range(len(paths)):
        if i == constraint['agent']:
            continue
        curr = get_location(paths[i], constraint['timestep'])
        prev = get_location(paths[i], constraint['timestep'] - 1)
        if len(constraint['loc']) == 1:  # vertex constraint
            if constraint['loc'][0] == curr:
                rst.append(i)
        else:  # edge constraint
            if constraint['loc'][0] == prev or constraint['loc'][1] == curr \
                    or constraint['loc'] == [curr, prev]:
                rst.append(i)
    return rst


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    if len(constraint_table) <= next_time:
        return False

    for constraint in constraint_table[next_time]:
        if constraint['positive']:  # positive constraint
            if constraint['loc'][0] != next_loc:
                return True
        else:  # negative constraint
            if len(constraint['loc']) == 1:  # vertex constraint
                if constraint['loc'][0] == next_loc:
                    return True
            else:  # edge constraint
                if constraint['loc'] == [curr_loc, next_loc]:
                    return True

    return False


def get_num_collisions(curr_loc, next_loc, next_time, paths):
    rst = 0
    for path in paths:
        if len(path) <= next_time:
            continue
        if path[next_time] == next_loc:
            rst += 1  # vertex conflict
        if next_time == 0:
            continue
        if path[next_time - 1] == next_loc and path[next_time] == curr_loc:
            rst += 1  # edge conflict
    return rst


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['collisions'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, _, curr = heapq.heappop(open_list)
    return curr


def delete_node(open_list, node):
    open_list.delete((node['g_val'] + node['h_val'], node['collisions'], node['h_val'], node['loc'], node))


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    if n1['g_val'] + n1['h_val'] == n2['g_val'] + n2['h_val']:
        return n1['collisions'] < n2['collisions']
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints, paths):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
        paths       - paths of other robots to try to avoid if possible,
                      but will not incur extra cost to do so
    """

    constraint_table = build_constraint_table(constraints, agent)
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'collisions': 0, 'timestep': 0, 'parent': None}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        if curr['loc'] == goal_loc and curr['timestep'] >= earliest_goal_timestep:
            found = True
            if curr['timestep'] + 1 < len(constraint_table):
                for t in range(curr['timestep'] + 1, len(constraint_table)):
                    if is_constrained(goal_loc, goal_loc, t, constraint_table):
                        found = False
                        earliest_goal_timestep = t + 1
                        break
            if found:
                return get_path(curr)
        for dir in range(5):
            next_loc = move(curr['loc'], dir)
            if my_map[next_loc[0]][next_loc[1]] or is_constrained(curr['loc'], next_loc, curr['timestep'] + 1,
                                                                  constraint_table):
                continue
            next_collisions = curr['collisions'] + get_num_collisions(
                curr['loc'], next_loc, curr['timestep'] + 1, paths)
            next = {'loc': next_loc, 'g_val': curr['g_val'] + 1,
                    'h_val': h_values[next_loc], 'collisions': next_collisions,
                    'timestep': curr['timestep'] + 1, 'parent': curr}
            if (next['loc'], next['timestep']) in closed_list:
                existing_node = closed_list[(next['loc'], next['timestep'])]
                if compare_nodes(next, existing_node):
                    closed_list[(next['loc'], next['timestep'])] = next
                    # delete_node(open_list, existing_node)
                    push_node(open_list, next)
            else:
                closed_list[(next['loc'], next['timestep'])] = next
                push_node(open_list, next)

    return None  # Failed to find solutions
