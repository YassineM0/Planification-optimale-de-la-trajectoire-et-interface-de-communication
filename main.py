# main.py
from map_data import nodes, graph
from astar import astar
from trajectory import generate_trajectory

def main():
    start = 'A'
    goal = 'R'

    path = astar(graph, nodes, start, goal)

    if not path:
        print("Aucun chemin trouvé.")
        return

    print("Plus court chemin trouvé :")
    print(" -> ".join(path))

    trajectory = generate_trajectory(
        path,
        nodes,
        v_max=1.0,
        a_max=0.5,
        dt=0.1,
        samples_per_segment=20,
    )

    print(f"Points de trajectoire : {len(trajectory)}")
    print("Aperçu (t, x, y, v, omega) :")
    for sample in trajectory[:10]:
        print(
            f"{sample['t']:.2f}, {sample['x']:.2f}, {sample['y']:.2f}, "
            f"{sample['v']:.2f}, {sample['omega']:.2f}"
        )

if __name__ == "__main__":
    main()
