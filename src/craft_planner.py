import json
from collections import namedtuple, defaultdict, OrderedDict, deque
from queue import PriorityQueue
from timeit import default_timer as time

Recipe = namedtuple('Recipe', ['name', 'check', 'effect', 'cost'])
SearchNode = namedtuple('SearchNode', ['state', 'causal_action'])



class State(OrderedDict):
    """ This class is a thin wrapper around an OrderedDict, which is simply a dictionary which keeps the order in
        which elements are added (for consistent key-value pair comparisons). Here, we have provided functionality
        for hashing, should you need to use a state as a key in another dictionary, e.g. distance[state] = 5. By
        default, dictionaries are not hashable. Additionally, when the state is converted to a string, it removes
        all items with quantity 0.

        Use of this state representation is optional, should you prefer another.
    """

    def __key(self):
        return tuple(self.items())

    def __hash__(self):
        return hash(self.__key())

    def __lt__(self, other):
        return self.__key() < other.__key()

    def copy(self):
        new_state = State()
        new_state.update(self)
        return new_state

    def __str__(self):
        return str(dict(item for item in self.items() if item[1] > 0))


def make_checker(rule):
    # Implement a function that returns a function to determine whether a state meets a
    # rule's requirements. This code runs once, when the rules are constructed before
    # the search is attempted.
    # print(rule.items())

    items_required = None
    if 'Requires' in rule.keys():
        items_required = rule.get('Requires').keys()

    items_consumed = rule.get('Consumes')

    def check(state):
        # This code is called by graph(state) and runs millions of times.
        # Tip: Do something with rule['Consumes'] and rule['Requires'].
        if items_required:
            for item in items_required:
                # Missing a required, non-consumed, item
                if state[item] < 1:
                    return False

        if items_consumed:
            for item, count in items_consumed.items():
                # Not enough of the consumable items to craft
                if state[item] < count:
                    return False

        return True

    return check


def make_effector(rule):
    # Implement a function that returns a function which transitions from state to
    # new_state given the rule. This code runs once, when the rules are constructed
    # before the search is attempted.
    items_produced = rule.get('Produces')
    items_consumed = rule.get('Consumes')

    def effect(state):
        # This code is called by graph(state) and runs millions of times
        # Tip: Do something with rule['Produces'] and rule['Consumes'].
        next_state = state.copy()
        if items_produced:
            for item, count in items_produced.items():
                next_state[item] += count

        if items_consumed:
            for item, count in items_consumed.items():
                next_state[item] -= count

        return next_state

    return effect


def make_goal_checker(goal):
    # Implement a function that returns a function which checks if the state has
    # met the goal criteria. This code runs once, before the search is attempted.
    items_required = goal.items()

    def is_goal(state):
        # This code is used in the search process and may be called millions of times.
        for item, count in items_required:
            if state[item] < count:
                return False
        return True

    return is_goal


def graph(state):
    # Iterates through all recipes/rules, checking which are valid in the given state.
    # If a rule is valid, it returns the rule's name, the resulting state after application
    # to the given state, and the cost for the rule.
    for r in all_recipes:
        if r.check(state):
            yield (r.name, r.effect(state), r.cost)


def heuristic(state):
    # Implement your heuristic here!
    # Value of state = (1 + (required - fufilled)/required) * Some constant

    return 1

def search(graph, state, is_goal, limit, heuristic):
    """


        Return:
            A list of tuples (state, recipe_name) that show the path taken to reach goal
            First element will be (state, None) as no recipe is needed to get to inital state.
    """

    start_time = time()

    # Implement your search here! Use your heuristic here!
    # When you find a path to the goal return a list of tuples [(state, action)]
    # representing the path. Each element (tuple) of the list represents a state
    # in the path and the action that took you to this state

    # A node in the search space is the (state, action) tuple
    distance    = 0
    node = SearchNode(state, None)

    visited_set = set()

    # A map of nodes and their parents
    parent_map  = {node: None}
    result_path = []

    # An element of the priority queue (distance_remaining, (node, distance_traveled))
    fringe      = PriorityQueue()
    fringe.put((distance, (node, distance)))

    goal = None

    while time() - start_time < limit:

        while not fringe.empty():
            distance, fringe_node = fringe.get()
            #print(fringe_node)

            if is_goal(fringe_node[0].state):
                goal = fringe_node[0]
                break

            visit_valid_aStar(graph, visited_set, parent_map, fringe, fringe_node, heuristic)

        # Search has completed unsucessfuly
        if not goal:
            break

        while goal:
            result_path.append(goal)
            goal = parent_map[goal]

        result_path = result_path[::-1]
        return result_path


    # Failed to find a path
    print(time() - start_time, 'seconds.')
    print("Failed to find a path from", state, 'within time limit.')
    return None

def visit_valid_aStar(graph, visited, parent_map, storage, storage_node, heu):
    visited.add(storage_node[0])
    visit_count = 0

    successor_states = graph(storage_node[0].state)
    for rule, next_state, next_cost in successor_states:
        c_node = SearchNode(next_state, rule)
        if c_node in visited:
            continue

        visited.add(c_node)
        # Push onto fringe, child and distance to goal
        # find the estimated distance to goal
        h = heu(c_node.state)
        # actual distance is stored as part of the item
        storage.put((storage_node[1] + h, (c_node, storage_node[1] + next_cost)))
        visit_count += 1

        # keep track of parent of s[0]
        parent_map[c_node] = storage_node[0]
    return visit_count


def aStarSearch(graph, state, is_goal, limit, heuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    while time() - start_time < limit:

        visit_valid_aStar(graph, visited_set, parent_map, fringe, start, distance, heuristic)

        while not (fringe.isEmpty()):
            current, distance = fringe.get()

            if is_goal(current[0]):
                goal = current[0]
                break
        visit_valid_aStar(graph, visited_set, parent_map, fringe, current[0], distance, heuristic)

        parent = parent_map[goal]
        while parent:
            directions.push(parent[1])
            parent = parent_map[parent[0]]

        while not (directions.isEmpty()):
            result.append(directions.pop())

        return result

    # Failed to find a path
    print(time() - start_time, 'seconds.')
    print("Failed to find a path from", state, 'within time limit.')
    return None

if __name__ == '__main__':
    with open('Crafting.json') as f:
        Crafting = json.load(f)

    # # List of items that can be in your inventory:
    # print('All items:', Crafting['Items'])
    #
    # # List of items in your initial inventory with amounts:
    # print('Initial inventory:', Crafting['Initial'])
    #
    # # List of items needed to be in your inventory at the end of the plan:
    # print('Goal:',Crafting['Goal'])
    #
    # # Dict of crafting recipes (each is a dict):
    # print('Example recipe:','craft stone_pickaxe at bench ->',Crafting['Recipes']['craft stone_pickaxe at bench'])

    # Build rules
    all_recipes = []
    for name, rule in Crafting['Recipes'].items():
        checker = make_checker(rule)
        effector = make_effector(rule)
        recipe = Recipe(name, checker, effector, rule['Time'])
        all_recipes.append(recipe)

    # Create a function which checks for the goal
    is_goal = make_goal_checker(Crafting['Goal'])

    # Initialize first state from initial inventory
    state = State({key: 0 for key in Crafting['Items']})
    state.update(Crafting['Initial'])

    # Search for a solution
    resulting_plan = search(graph, state, is_goal, 5, heuristic)
    # resulting_plan = aStarSearch(graph, state, is_goal, 5, heuristic)

    if resulting_plan:
        # Print resulting plan
        for state, action in resulting_plan:
            print('\t',state)

