# astar.py
import heapq
from utils import euclidean_distance

def astar(graph, nodes, start, goal):
    """
    Algorithme A* pour le plus court chemin
    graph : dictionnaire des connexions
    nodes : positions des nœuds
    start : nœud de départ
    goal  : nœud d'arrivée
    """

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}

    g_score = {node: float('inf') for node in graph}
    g_score[start] = 0

    f_score = {node: float('inf') for node in graph}
    f_score[start] = euclidean_distance(nodes[start], nodes[goal])

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in graph[current]:
            tentative_g = g_score[current] + euclidean_distance(
                nodes[current], nodes[neighbor]
            )

            if tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + euclidean_distance(
                    nodes[neighbor], nodes[goal]
                )
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]
