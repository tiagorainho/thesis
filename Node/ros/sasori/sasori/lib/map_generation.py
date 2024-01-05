import numpy as np

MAP_POINT_PER_METER = 25

def random_map(delta_x: float, delta_y: float, delta_z: float, n: float = None, points_per_meter = MAP_POINT_PER_METER) -> np.ndarray:

        if n is None:
            n = MAP_POINT_PER_METER * delta_x * delta_y

        return np.random.uniform([-delta_x/2, -delta_y/2, -delta_z/2], [delta_x/2, delta_y/2, delta_z/2], size=(n, 3))

def map_mountain(delta_x: float, delta_y: float, delta_z: float, n: float = None) -> np.ndarray:
    points = random_map(delta_x=delta_x, delta_y=delta_y, delta_z=delta_z, n=n)

    mountains = [
        (1, 1, 4, 2),
        (-5, -5, 2, 3),
        (-3, 2, 3, 2),
        (6, 4, 3, 3)
    ]

    for p in points:
        x, y, _ = p
        z = 0

        for mountain_x, mountain_y, mountain_z, mountain_delta_z in mountains:
            new_x = x + mountain_x
            new_y = y + mountain_y
            x = new_x
            y = new_y
            # z += mountain_z / (new_x*new_x + new_x*new_y + new_y*new_y + mountain_delta_z)

            z += -(0.01 * (x-1) * (x-1) + 0.01 * (y-1) * (y-1))
        p[2] = z

    return points