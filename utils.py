# utils.py
import math

def euclidean_distance(p1, p2):
    """
    Calcule la distance euclidienne entre deux points 2D
    """
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
