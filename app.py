import random
import time
import heapq


class Node:
    """
    A single Node carries information about the current state of the
    grid and the list of moves it takes to get there
    """
    def __init__(
            self,
            a_pos: tuple,
            b_pos: tuple,
            c_pos: tuple,
            agent_pos: tuple,
            moves_list: list,
    ):
        self.a_pos = a_pos
        self.b_pos = b_pos
        self.c_pos = c_pos
        self.agent_pos = agent_pos
        self.moves_list = moves_list


class Controller:
    def __init__(self, map_size: int):
        self.map_size = map_size

    # This takes in the current board positions and a move and returns the
    # new board positions after the move
    def move_agent(
            self,
            agent_pos: tuple,
            block_pos: list,
            move: str
    ) -> dict:
        a_pos = block_pos[0]
        b_pos = block_pos[1]
        c_pos = block_pos[2]

        prev_agent_pos = agent_pos
        new_agent_pos = None

        if move == 'left':
            new_agent_pos = (agent_pos[0] - 1, agent_pos[1])
        elif move == 'right':
            new_agent_pos = (agent_pos[0] + 1, agent_pos[1])
        elif move == 'up':
            new_agent_pos = (agent_pos[0], agent_pos[1] + 1)
        elif move == 'down':
            new_agent_pos = (agent_pos[0], agent_pos[1] - 1)

        if new_agent_pos == a_pos:
            a_pos = prev_agent_pos
        elif new_agent_pos == b_pos:
            b_pos = prev_agent_pos
        elif new_agent_pos == c_pos:
            c_pos = prev_agent_pos

        return {
            'block_pos': [a_pos, b_pos, c_pos],
            'agent_pos': new_agent_pos
        }

    # This takes in the agent's position and based on the map size returns
    # an array of all possible moves that can be made from that position
    def get_possible_moves(self, agent_pos: tuple) -> list:
        max_xy = self.map_size - 1
        agent_x = agent_pos[0]
        agent_y = agent_pos[1]

        if agent_x == 0 and agent_y == 0:
            return ['right', 'up']
        elif agent_x == max_xy and agent_y == max_xy:
            return ['left', 'down']
        elif agent_y == 0 and agent_x == max_xy:
            return ['left', 'up']
        elif agent_x == 0 and agent_y == max_xy:
            return ['right', 'down']
        elif agent_x == max_xy:
            return ['left', 'up', 'down']
        elif agent_y == max_xy:
            return ['left', 'right', 'down']
        elif agent_x == 0:
            return ['right', 'up', 'down']
        elif agent_y == 0:
            return ['up', 'left', 'right']
        else:
            return ['up', 'down', 'left', 'right']


def dfs(map_size: int, starting_node: Node, goal_pos: list):
    controller = Controller(map_size)
    node = starting_node
    node_count = 0

    while True:
        potential_moves = controller.get_possible_moves(node.agent_pos)
        # Randomly choose a move and go
        move = random.choice(potential_moves)
        # New list of moves it takes to get to the new position
        moves_list = node.moves_list + [move]
        new_pos = controller.move_agent(node.agent_pos, [node.a_pos, node.b_pos, node.c_pos], move)
        node = Node(
            a_pos=new_pos['block_pos'][0],
            b_pos=new_pos['block_pos'][1],
            c_pos=new_pos['block_pos'][2],
            agent_pos=new_pos['agent_pos'],
            moves_list=moves_list
        )
        node_count += 1

        # If the new move leads to the goal position, return the list of moves
        if new_pos['block_pos'] == goal_pos:
            return {
                'moves': moves_list,
                'node_count': node_count
            }


def bfs(map_size: int, starting_node: Node, goal_pos: list):
    controller = Controller(map_size)
    node_count = 0
    depth = 0
    # Initialize the queue with the start position
    queue = [starting_node]

    while queue:
        node = queue.pop(0)
        node_count += 1

        if len(node.moves_list) - 1 > depth:
            depth += 1
            print(f'Depth: {depth}, Node count: {node_count}')

        if [node.a_pos, node.b_pos, node.c_pos] == goal_pos:
            return {
                'moves': node.moves_list,
                'node_count': node_count
            }
        else:
            possible_moves = controller.get_possible_moves(node.agent_pos)
            # Generate successors based on possible moves from the agent's position
            for move in possible_moves:
                new_pos = controller.move_agent(node.agent_pos, [node.a_pos, node.b_pos, node.c_pos], move)
                moves_list = node.moves_list + [move]

                queue.append(Node(
                    a_pos=new_pos['block_pos'][0],
                    b_pos=new_pos['block_pos'][1],
                    c_pos=new_pos['block_pos'][2],
                    agent_pos=new_pos['agent_pos'],
                    moves_list=moves_list
                ))


def ids(map_size: int, starting_node: Node, goal_pos: list):
    depth_limit = 0
    node_count = 0
    controller = Controller(map_size)
    stack = [starting_node]

    global depth_start_time
    depth_start_time = time.time()

    while stack:
        node = stack.pop()
        node_count += 1

        # Check if the current node is at the goal
        if [node.a_pos, node.b_pos, node.c_pos] == goal_pos:
            return {
                'moves': node.moves_list,
                'node_count': node_count
            }
        else:
            # Check if all the nodes at this depth limit has been checked
            if len(node.moves_list) == depth_limit and not stack:
                print(f'Depth: {depth_limit}, Node count: {node_count}')
                stack = [starting_node]
                depth_limit += 1
            # Only add new nodes to the queue if the current node isn't at the bottom of the tree
            elif len(node.moves_list) < depth_limit:
                possible_moves = controller.get_possible_moves(node.agent_pos)
                for move in possible_moves:
                    new_pos = controller.move_agent(node.agent_pos, [node.a_pos, node.b_pos, node.c_pos], move)
                    moves_list = node.moves_list + [move]

                    stack.append(Node(
                        a_pos=new_pos['block_pos'][0],
                        b_pos=new_pos['block_pos'][1],
                        c_pos=new_pos['block_pos'][2],
                        agent_pos=new_pos['agent_pos'],
                        moves_list=moves_list
                    ))


def a_star(map_size: int, starting_node: Node, goal_pos: list):
    # Heuristic function
    def manhattan_distance(block_positions: list) -> int:
        output = 0

        for index, val in enumerate(block_positions):
            x_diff = abs(goal_pos[index][0] - val[0])
            y_diff = abs(goal_pos[index][1] - val[1])
            output += (x_diff + y_diff)

        return output

    controller = Controller(map_size)
    node_count = 0
    # This is used as a secondary comparator in case the manhattan distance is the same
    pq_sequence = 0
    depth = 0
    # Initialize the queue
    queue = [(
        manhattan_distance([starting_node.a_pos, starting_node.b_pos, starting_node.c_pos]),
        pq_sequence,
        starting_node
    )]
    # Turn the queue into a priority queue
    heapq.heapify(queue)

    while queue:
        node = heapq.heappop(queue)[2]
        node_count += 1

        if len(node.moves_list) - 1 > depth:
            depth += 1
            print(f'Depth: {depth}, Node count: {node_count}')

        # Check if the node is at goal state
        if [node.a_pos, node.b_pos, node.c_pos] == goal_pos:
            return {
                'moves': node.moves_list,
                'node_count': node_count
            }
        else:
            possible_moves = controller.get_possible_moves(node.agent_pos)
            for index, move in enumerate(possible_moves):
                new_pos = controller.move_agent(node.agent_pos, [node.a_pos, node.b_pos, node.c_pos], move)
                moves_list = node.moves_list + [move]
                pq_sequence += 1

                heuristic = manhattan_distance(new_pos['block_pos'])

                heapq.heappush(queue, (
                    # The addition of depth ensures the algo doesn't just run around in a circle
                    heuristic + depth,
                    pq_sequence,
                    Node(
                        a_pos=new_pos['block_pos'][0],
                        b_pos=new_pos['block_pos'][1],
                        c_pos=new_pos['block_pos'][2],
                        agent_pos=new_pos['agent_pos'],
                        moves_list=moves_list
                    )
                ))


test_node = Node(
    a_pos=(0, 0),
    b_pos=(1, 0),
    c_pos=(2, 0),
    agent_pos=(3, 0),
    moves_list=[]
)

goal = [(1, 2), (1, 1), (1, 0)]

output = a_star(4, test_node, goal)
print(f'Final action sequence: {output["moves"]}')
print(f'Final depth reached: {len(output["moves"])}')
print(f'Number of nodes expanded: {output["node_count"]}')
