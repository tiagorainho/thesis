from __future__ import annotations
from typing import List, Tuple, Union
from collections.abc import Iterable
from math import dist
from queue import PriorityQueue
import numpy as np


class Node:
    value: Tuple[float]
    parent: Node
    depth: int
    cost: float
    distance_to_goal: float
    heuristic: float

    def __init__(self, value, cost: float = 0, parent: Node = None):
        self.value = value
        self.parent = parent
        self.cost = cost
        self.depth = 0 if self.parent is None else self.parent.depth + 1

    @staticmethod
    def heuristics_a_star(node, goal):
        node.distance_to_goal = dist(node.value, goal)
        node.heuristic = node.cost + node.distance_to_goal

    @staticmethod
    def heuristics_bfs(node):
        node.heuristic = node.cost

    def __eq__(self, other):
        return self.value == other.value

    def __lt__(self, other: Node):
        return self.heuristic < other.heuristic

    def __repr__(self):
        return f"{self.value} -> {self.heuristic}"

    def __hash__(self):
        return hash(tuple(self.value))


class Graph:

    def __init__(self, pc):
        self.pc = pc

    @staticmethod
    def generic_a_star(
        start,
        goal,
        check_done=lambda node: bool,
        acceptance_condition=lambda node, neighbor: True,
        ignore_condition=lambda node: False,
        get_neighbors=lambda node: Iterable,
        apply_heuristic=lambda node, goal: Node.heuristics_a_star(node=node, goal=goal)
    ) -> Union[List[Tuple[float]], None]:

        closed_nodes = set()

        queue = PriorityQueue()
        start_node = Node(value=tuple(start), cost=0)
        apply_heuristic(node=start_node, goal=goal)
        queue.put((0, start_node))
        # closed_nodes.add(start_node)

        while not queue.empty():
            node = queue.get()[1]
            closed_nodes.add(node)

            if check_done(node):
                path = [goal]
                current_node = node
                while current_node is not None:
                    path.append(current_node.value)
                    current_node = current_node.parent
                return path[::-1]

            if ignore_condition(node):
                continue

            current_total_cost = node.parent.cost if node.parent is not None else 0

            neighbors = get_neighbors(node)
            for neighbor in neighbors:
                neighbor_node = Node(
                    value=tuple(neighbor),
                    cost=current_total_cost + dist(node.value, neighbor),
                    parent=node
                )
                apply_heuristic(node=neighbor_node, goal=goal)

                if neighbor_node in closed_nodes:
                    continue

                if not acceptance_condition(node, neighbor_node):
                    continue

                queue.put((neighbor_node.heuristic, neighbor_node))
                # closed_nodes.add(neighbor_node)

        return None

    @staticmethod
    def rrt(
            start,
            check_done=lambda node: bool,
            get_neighbors=lambda node, closed_nodes: Iterable,
            acceptance_condition=lambda node, neighbor: True,
            ignore_condition=lambda node, opened_nodes, closed_nodes: False,
            apply_heuristic=lambda node: Node.heuristics_bfs(node=node),
    ):
        closed_points = set()
        opened_nodes = set()

        queue = PriorityQueue()
        start_node = Node(value=tuple(start))
        apply_heuristic(node=start_node)
        queue.put((0, start_node))
        i = 0
        while not queue.empty():
            i += 1
            node = queue.get()[1]

            if check_done(node.value):
                path = []
                current_node = node
                while current_node is not None:
                    path.append(current_node.value)
                    current_node = current_node.parent
                print("--------> " + str(i))
                return path[::-1]

            if ignore_condition(node, opened_nodes, closed_points):
                continue

            current_total_cost = node.parent.cost if node.parent is not None else 0

            neighbors = get_neighbors(node, closed_points)
            for neighbor in neighbors:
                neighbor_node = Node(
                    value=tuple(neighbor),
                    cost=current_total_cost + dist(node.value, neighbor),
                    parent=node
                )
                apply_heuristic(node=neighbor_node)

                if neighbor_node in opened_nodes:
                    continue

                opened_nodes.add(neighbor_node)

                if not acceptance_condition(node, neighbor_node):
                    closed_points.add(node.value)
                    continue

                # add to queue
                queue.put((neighbor_node.heuristic, neighbor_node))

                # add to closed nodes
            closed_points.add(node.value)
        print("--------> " + str(i))
        return None

    @staticmethod
    def generic_bfs(
            start,
            check_done=lambda node: bool,
            get_neighbors=lambda node: Iterable,
            acceptance_condition=lambda node, neighbor: True,
            ignore_condition=lambda node, opened_nodes, closed_nodes: False,
            apply_heuristic=lambda node: Node.heuristics_bfs(node=node),
    ) -> Union[List[Tuple[float, float, float]], None]:

        closed_nodes = set()
        opened_nodes = set()

        queue = PriorityQueue()
        start_node = Node(value=tuple(start))
        apply_heuristic(node=start_node)
        queue.put((0, start_node))
        while not queue.empty():
            node = queue.get()[1]

            if check_done(node):
                path = []
                current_node = node
                while current_node is not None:
                    path.append(current_node.value)
                    current_node = current_node.parent
                return path[::-1]

            if ignore_condition(node, opened_nodes, closed_nodes):
                continue

            current_total_cost = 0 if node is None else node.cost

            neighbors = get_neighbors(node)
            for neighbor in neighbors:
                neighbor_node = Node(
                    value=tuple(neighbor),
                    cost=current_total_cost + dist(node.value, neighbor),
                    parent=node
                )

                apply_heuristic(node=neighbor_node)

                if neighbor_node in opened_nodes:
                    continue

                opened_nodes.add(neighbor_node)

                if not acceptance_condition(node, neighbor_node):
                    closed_nodes.add(node)
                    continue

                # add to queue
                queue.put((neighbor_node.heuristic, neighbor_node))

                # add to closed nodes
            closed_nodes.add(node)
        return None
